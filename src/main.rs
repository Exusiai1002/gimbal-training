#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};
use embedded_can::Id;
// This rig uses GM6020 voltage commands for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FF;
// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);
const FEEDBACK_TIMEOUT_TICKS: u16 = 20;

// ============================================================================
// 教具配置（明天现场标定后改这一节）
// ============================================================================

/// 一圈 8192 个刻度 → 每个刻度 = 2π/8192 rad。
const ANGLE_RAD_PER_COUNT: f32 = core::f32::consts::TAU / 8192.0;
/// 参考姿态（yaw 正前方 / pitch 水平）的机械角原始计数。
/// `None` = 还没标定：不自动回正、限位也不生效，只保持上电姿态。
const YAW_REFERENCE_RAW: Option<u16> = Some(3422);
const PITCH_REFERENCE_RAW: Option<u16> = Some(4153);
/// 安装方向：反馈正方向与关节正方向相反时改成 -1.0。
const YAW_DIRECTION: f32 = 1.0;
const PITCH_DIRECTION: f32 = 1.0;
/// pitch 机械限位（rad，相对参考姿态）。实测 a 极限 2.97~3.43 rad，参考 4153(≈3.185 rad)，
/// 换算关节角约 -0.215~+0.245 rad；下面留约 0.02 rad 余量。
const PITCH_MIN_RAD: f32 = -0.195;
const PITCH_MAX_RAD: f32 = 0.225;
const YAW_MIN_RAD: Option<f32> = None;
const YAW_MAX_RAD: Option<f32> = None;

// ============================================================================
// 遥控 / 回正 / 故障阈值
// ============================================================================

/// 摇杆死区（归一化 ±5%）。
const STICK_DEADBAND: f32 = 0.05;
/// 拨满摇杆时目标角度的变化速度（rad/s）。
const YAW_MAX_TARGET_RATE_RAD_S: f32 = 1.5;
const PITCH_MAX_TARGET_RATE_RAD_S: f32 = 1.0;
/// 回正：限目标速度、限输出幅值、限每周期输出变化。
const PREPARE_MAX_TARGET_RPM: f32 = 150.0;
const PREPARE_MAX_OUTPUT_RAW: f32 = 12000.0;
const PREPARE_MAX_OUTPUT_STEP_RAW: f32 = 30.0;
/// Running 阶段每 1 ms 允许的电压变化量（抑制高频振动/蜂鸣）。
const RUNNING_MAX_OUTPUT_STEP_RAW: f32 = 150.0;
/// 回正完成判据与超时。
const SETTLE_ERR_RAD: f32 = 0.03;
const SETTLE_SPEED_RPM: f32 = 20.0;
const SETTLE_TICKS: u16 = 100;
const PREPARE_TIMEOUT_TICKS: u16 = 5000;
/// 回正起步的这段时间不做故障判定（允许电机从静止加速）。
const PREPARE_FAULT_GRACE_TICKS: u16 = 500;
/// 疑似堵转：目标速度 ≥20 rpm 且位置误差 ≥0.10 rad、输出 ≥2000、实际速度 ≈0，连续 800 ms。
const STALL_TARGET_RPM: f32 = 20.0;
const STALL_OUTPUT_RAW: f32 = 2000.0;
const STALL_SPEED_RPM: f32 = 0.5;
/// 堵转还要求“位置误差确实存在”（rad）：防止重载静止保持（输出很大但误差小）被误判。
const STALL_ERR_RAD: f32 = 0.10;
const STALL_TICKS: u16 = 800;
/// 持续高频振荡：250 ms 窗口内 ≥2 个完整往返（≥8 Hz）、峰峰值 ≥0.04 rad、连续 2 窗口（0.5 s）。
const OSC_WINDOW_TICKS: u16 = 250;
const OSC_MIN_CYCLES: u16 = 2;
const OSC_MIN_P2P_RAD: f32 = 0.04;
const OSC_MAX_WINDOWS: u8 = 2;
const OSC_SIGN_EPS_RAD: f32 = 0.01;
/// 限频打印 CAN 编码结果的周期（约 1 s），避免 1 kHz 刷屏。
const TX_LOG_PERIOD_TICKS: u16 = 1000;
const YAW_MOTOR_ID: u8 = 1;
const PITCH_MOTOR_ID: u8 = 2;
const MOTOR_FEEDBACK_ID_BASE: u16 = 0x204;
/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

/// Feedback and voltage command for one motor.
#[derive(Default)]
pub struct Motor {
    /// Single-turn motor shaft angle in rad; turns are not counted.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
}

impl Motor {
    /// Clear the voltage command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    pub fn read(&mut self, data: &[u8; 8]) {
        let (angle_rad, speed_rpm, current_raw) = decode_feedback(data);
        self.angle_rad = angle_rad;
        self.speed_rpm = speed_rpm;
        self.current_raw = current_raw;
    }
}
fn decode_feedback(data: &[u8; 8]) -> (f32, f32, i16) {
    let angle_raw = u16::from_be_bytes([data[0], data[1]]) & 0x1FFF;
    (
        angle_raw as f32 * (core::f32::consts::TAU / 8192.0),
        i16::from_be_bytes([data[2], data[3]]) as f32,
        i16::from_be_bytes([data[4], data[5]]),
    )
}
fn clamp_voltage(value: f32) -> i16 {
    let limited = if value.is_nan() {
        0.0
    } else if value > 25_000.0 {
        25_000.0
    } else if value < -25_000.0 {
        -25_000.0
    } else {
        value
    };
    if limited >= 0.0 {
        (limited + 0.5) as i16
    } else {
        (limited - 0.5) as i16
    }
}

fn clamp_symmetric(value: f32, limit: f32) -> f32 {
    if value.is_nan() {
        0.0
    } else if value > limit {
        limit
    } else if value < -limit {
        -limit
    } else {
        value
    }
}
fn encode_voltage_command(data: &mut [u8; 8], motor_id: u8, command_voltage_raw: f32) {
    if motor_id == 0 || motor_id > 4 {
        return;
    }
    let slot = (motor_id - 1) as usize * 2;
    let raw = clamp_voltage(command_voltage_raw);
    data[slot..slot + 2].copy_from_slice(&raw.to_be_bytes());
}

// ============================================================================
// 通用小工具
// ============================================================================

/// 普通限幅（带上限和下限），NaN 归零。
fn clamp_value(value: f32, min: f32, max: f32) -> f32 {
    if value.is_nan() {
        0.0
    } else if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// 把角度折回 (-π, π]：用于 8191→0 的回绕，避免出现接近 2π 的假误差。
fn wrap_pi(angle: f32) -> f32 {
    let mut wrapped = angle;
    while wrapped > core::f32::consts::PI {
        wrapped -= core::f32::consts::TAU;
    }
    while wrapped <= -core::f32::consts::PI {
        wrapped += core::f32::consts::TAU;
    }
    wrapped
}

/// 摇杆归一化输入去死区：死区外从 0 线性重映射到 ±1。
fn apply_deadband(input: f32) -> f32 {
    let magnitude = input.abs();
    if magnitude <= STICK_DEADBAND {
        0.0
    } else {
        let scaled = (magnitude - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
        clamp_value(scaled, 0.0, 1.0) * if input < 0.0 { -1.0 } else { 1.0 }
    }
}

// ---------------------------------------------------------------------------
// 遥控器拨杆：物理左右 ↔ 结构体字段的唯一映射点
// ---------------------------------------------------------------------------
//
// 作业约定：右上拨杆 = enable（中位 3）/ disable（下位 2），上位 1 不使能；
// 左上拨杆用于 Error 的人工恢复确认。
// 手册表格与附录代码对 S1/S2 位分配自相矛盾；若明天上机发现右上拨杆读到的是
// `left_switch`，只改下面两个函数的字段（不要只改一半）。

/// 物理右上拨杆。
fn right_switch(remote: &dm_mc02::Dr16) -> u8 {
    remote.right_switch
}

/// 物理左上拨杆。
fn left_switch(remote: &dm_mc02::Dr16) -> u8 {
    remote.left_switch
}
/// 单环 PID 的配置项：现场调参改这里。
///
/// 用字段名而不是“第几个参数”，避免记错位置：
/// - `kp`：比例增益。位置环单位 rpm/rad；速度环单位 电压原始值/rpm。
/// - `ki`：积分增益。默认 0；有稳态偏差（如 pitch 下垂）才加小量。
/// - `kd`：微分增益。默认 0；超调大才加，对噪声敏感。
/// - `maxout`：本环输出的对称上限。位置环 = 目标转速上限(rpm)；速度环 = 台架电压上限。
/// - `integral_limit`：积分累计限幅，防积分饱和。
#[derive(Clone, Copy)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub maxout: f32,
    pub integral_limit: f32,
}

/// PID 单环控制器：参数来自 `PidConfig`，积分/微分历史保存在内部。
#[derive(Default)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub maxout: f32,
    pub integral_limit: f32,
    pub last_output: f32,
    integral: f32,
    previous_error: f32,
    started: bool,
}

impl Pid {
    pub fn new(config: PidConfig) -> Self {
        Self {
            kp: config.kp,
            ki: config.ki,
            kd: config.kd,
            maxout: config.maxout,
            integral_limit: config.integral_limit,
            last_output: 0.0,
            integral: 0.0,
            previous_error: 0.0,
            started: false,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.started = false;
        self.last_output = 0.0;
    }

    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        self.calculate_error(target - measured, dt_s)
    }

    pub fn calculate_error(&mut self, error: f32, dt_s: f32) -> f32 {
        let error = if error.is_nan() { 0.0 } else { error };

        if !self.started {
            self.previous_error = error;
            self.started = true;
        }

        if self.ki != 0.0 {
            self.integral += error * dt_s;
            self.integral = clamp_symmetric(self.integral, self.integral_limit);
        } else {
            self.integral = 0.0;
        }

        let derivative = if dt_s > 0.0 {
            (error - self.previous_error) / dt_s
        } else {
            0.0
        };

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        self.previous_error = error;

        let limited = clamp_symmetric(output, self.maxout);
        self.last_output = limited;
        limited
    }
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
#[derive(Default)]
pub struct Axis {
    pub motor: Motor,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, voltage counts out.
    pub speed_pid: Pid,
    pub feedback_age_ticks: u16,

    // ---- 教具配置（由 Control::default 按轴填入）----
    /// 参考姿态的机械角原始计数；None = 未标定。
    pub reference_raw: Option<u16>,
    /// 安装方向 ±1。
    pub direction: f32,
    /// 关节角限位（rad，相对参考姿态）；None = 按回绕处理。
    pub min_rad: Option<f32>,
    pub max_rad: Option<f32>,
    /// 上一周期电压指令，供回正时的输出斜率限制。
    pub previous_command_raw: f32,

    // ---- 故障检测状态（Tuning Studio 可看）----
    /// 疑似堵转已持续周期数。
    pub stall_ticks: u16,
    /// 当前统计窗口内的过零次数。
    pub osc_crossings: u16,
    /// 当前统计窗口已累计周期数。
    pub osc_window_ticks: u16,
    /// 连续超阈值窗口数。
    pub osc_windows_over: u8,
    /// 最近窗口估计的振荡频率（Hz）。
    pub osc_freq_hz: f32,

    osc_previous_sign: i8,
    osc_min_error: f32,
    osc_max_error: f32,
}

impl Axis {
    /// 电机单圈机械角 → 关节角（rad，相对参考姿态，范围 (-π, π]）。
    pub fn joint_angle_rad(&self) -> f32 {
        let reference_rad = self
            .reference_raw
            .map(|raw| raw as f32 * ANGLE_RAD_PER_COUNT)
            .unwrap_or(0.0);
        wrap_pi((self.motor.angle_rad - reference_rad) * self.direction)
    }

    /// 目标角规范化：已标定且有限位 → 夹到 [min,max]；否则回绕到 (-π, π]。
    /// 未标定时限位没有意义，否则会把“当前角”夹进限位、反而朝限位运动。
    pub fn sanitize_target(&self, target: f32) -> f32 {
        match (self.min_rad, self.max_rad, self.reference_raw) {
            (Some(min), Some(max), Some(_)) => clamp_value(target, min, max),
            _ => wrap_pi(target),
        }
    }

    /// 位置环误差（rad，关节坐标）：有限位的轴用直接差，其余用最短路径。
    pub fn control_error(&self) -> f32 {
        let error = self.target_angle_rad - self.joint_angle_rad();
        match (self.min_rad, self.max_rad, self.reference_raw) {
            (Some(_), Some(_), Some(_)) => error,
            _ => wrap_pi(error),
        }
    }

    /// 重置一次控制会话：目标角给定，速度目标/PID 历史/输出斜率起点清零。
    pub fn reset_control(&mut self, target_angle_rad: f32) {
        self.target_angle_rad = self.sanitize_target(target_angle_rad);
        self.target_rpm = 0.0;
        self.position_pid.reset();
        self.speed_pid.reset();
        self.motor.disable();
        self.previous_command_raw = 0.0;
        self.reset_detection();
    }

    /// 清理故障检测状态（不代表故障消失，只是重新计数）。
    fn reset_detection(&mut self) {
        self.stall_ticks = 0;
        self.osc_crossings = 0;
        self.osc_window_ticks = 0;
        self.osc_windows_over = 0;
        self.osc_freq_hz = 0.0;
        self.osc_previous_sign = 0;
        self.osc_min_error = f32::MAX;
        self.osc_max_error = f32::MIN;
    }

    /// 一个周期的故障判定：返回 Some 表示这一轴应进入 Error。
    fn update_fault_detection(&mut self) -> Option<AxisFault> {
        // 判据一：疑似堵转 —— 有运动需求、输出大、但实际速度持续很低。
        // 要求“有目标速度”是为了排除静止保持和零速度环调参（位置环 maxout=0）。
        // 有运动需求：目标转速大 **且** 位置误差确实存在（否则重力保持会被误判堵转）。
        let has_demand = self.target_rpm.abs() >= STALL_TARGET_RPM
            && self.control_error().abs() >= STALL_ERR_RAD;
        let has_effort = self.motor.command_voltage_raw.abs() >= STALL_OUTPUT_RAW;
        let is_slow = self.motor.speed_rpm.abs() <= STALL_SPEED_RPM;
        if has_demand && has_effort && is_slow {
            self.stall_ticks = self.stall_ticks.saturating_add(1);
            if self.stall_ticks >= STALL_TICKS {
                return Some(AxisFault::Stall);
            }
        } else {
            self.stall_ticks = 0;
        }

        // 判据二：持续高频振荡。围绕“窗口中心”统计过零（带偏置的振荡也能测到）：
        // 2 次过零 = 1 个完整往返；窗口内往返数 + 峰峰值 + 连续窗口数三者同时满足才判。
        let error = self.control_error();
        if error < self.osc_min_error {
            self.osc_min_error = error;
        }
        if error > self.osc_max_error {
            self.osc_max_error = error;
        }
        let center = 0.5 * (self.osc_min_error + self.osc_max_error);
        let sign = if error > center + OSC_SIGN_EPS_RAD {
            1
        } else if error < center - OSC_SIGN_EPS_RAD {
            -1
        } else {
            0
        };
        if sign != 0 {
            if self.osc_previous_sign != 0 && sign != self.osc_previous_sign {
                self.osc_crossings = self.osc_crossings.saturating_add(1);
            }
            self.osc_previous_sign = sign;
        }

        self.osc_window_ticks = self.osc_window_ticks.saturating_add(1);
        if self.osc_window_ticks >= OSC_WINDOW_TICKS {
            let cycles = self.osc_crossings / 2;
            let window_s = OSC_WINDOW_TICKS as f32 * 0.001;
            self.osc_freq_hz = cycles as f32 / window_s;
            let peak_to_peak = self.osc_max_error - self.osc_min_error;
            let window_is_bad = cycles >= OSC_MIN_CYCLES && peak_to_peak >= OSC_MIN_P2P_RAD;
            if window_is_bad {
                self.osc_windows_over = self.osc_windows_over.saturating_add(1);
            } else {
                self.osc_windows_over = 0;
            }

            // 开始下一个窗口。
            self.osc_crossings = 0;
            self.osc_window_ticks = 0;
            self.osc_min_error = f32::MAX;
            self.osc_max_error = f32::MIN;
            self.osc_previous_sign = 0;

            if self.osc_windows_over >= OSC_MAX_WINDOWS {
                self.osc_windows_over = 0;
                return Some(AxisFault::Oscillation);
            }
        }

        None
    }
}

/// 单轴的故障结论。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AxisFault {
    Stall,
    Oscillation,
}

/// 锁存的故障原因：不依赖 `State::Error` 保存，断联重连也不会清除。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    YawStall,
    PitchStall,
    YawOscillation,
    PitchOscillation,
}

/// State shared by every control tick.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
    /// 已锁存的故障；None = 当前没有未处理故障。
    pub fault: Option<Fault>,
    /// 回正已运行的周期数（1 ms）。
    pub prepare_ticks: u16,
    /// 两轴同时满足回正完成判据的连续周期数。
    pub settle_ticks: u16,
    /// 是否已经见过右上拨杆下拨（之后回中位才算一次新的使能）。
    pub enable_armed: bool,
    /// 上一周期的拨杆位置，用于识别“一次新的操作”。
    pub last_right_switch: u8,
    pub last_left_switch: u8,
    /// 恢复手势“已先下拨”记忆（拨杆下→上会经过中位，不能用相邻周期边沿）。
    pub recovery_armed: bool,
    /// 限频打印编码结果用的计数（约 1 s 一次）。
    pub transmit_log_ticks: u16,
}

/// Start in `Waiting` with no remote input.
/// 这里给两轴填教具配置和 PID 初值（初值只求“能安全动”，明天现场按 README 四调）。
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            yaw: Axis {
                reference_raw: YAW_REFERENCE_RAW,
                direction: YAW_DIRECTION,
                min_rad: YAW_MIN_RAD,
                max_rad: YAW_MAX_RAD,
                // 位置环：rad → rpm（maxout=0 时输出恒 0，用于单独调速度环）
                position_pid: Pid::new(PidConfig {
                    kp: 250.0,             // 位置环 P：300→250（回正接近目标更平滑，减少粘滑抖动）
                    ki: 0.0,               // 位置环 I：默认 0
                    kd: 0.0,               // 位置环 D：3.0→0（D 放大噪声，先撤；需要时最多 0.05~0.1）
                    maxout: 90.0,          // 输出上限=目标转速上限(rpm)；单独调速度环时改 0.0
                    integral_limit: 400.0, // 积分限幅
                }),
                // 速度环：rpm → 电压原始值（maxout 是台架输出上限）
                speed_pid: Pid::new(PidConfig {
                    kp: 80.0,               // 速度环 P：60→80（补 droop，减少对积分的依赖）
                    ki: 8.0,                // 速度环 I：20→8（回正接近目标时减少积分过冲/反向）
                    kd: 0.0,                // 速度环 D：一般保持 0
                    maxout: 5000.0,         // 台架电压上限（协议 ±25000，调试先小）
                    integral_limit: 300.0,  // 积分限幅：800→300
                }),
                osc_min_error: f32::MAX,
                osc_max_error: f32::MIN,
                ..Axis::default()
            },
            pitch: Axis {
                reference_raw: PITCH_REFERENCE_RAW,
                direction: PITCH_DIRECTION,
                min_rad: Some(PITCH_MIN_RAD),
                max_rad: Some(PITCH_MAX_RAD),
                position_pid: Pid::new(PidConfig {
                    kp: 1200.0,   // 位置环 P：1500→1200（回正接近目标更平滑）
                    ki: 0.0,      // 位置环 I：默认 0
                    kd: 0.0,      // 位置环 D：超调大再给 0.05~0.2
                    maxout: 70.0, // 目标转速上限：50→70（用 P 提供爬坡力，降低对积分的依赖）
                    integral_limit: 400.0, // 积分限幅
                }),
                speed_pid: Pid::new(PidConfig {
                    kp: 200.0,              // 速度环 P：300→200（降内环增益抑制蜂鸣，同时保住回正爬坡余量）
                    ki: 100.0,              // 速度环 I：200→100（回正接近目标时减少积分过冲/抖动）
                    kd: 0.0,                // 速度环 D：一般保持 0
                    maxout: 10000.0,        // 台架电压上限
                    integral_limit: 1500.0, // 积分限幅：3000→1500
                }),
                osc_min_error: f32::MAX,
                osc_max_error: f32::MIN,
                ..Axis::default()
            },
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            fault: None,
            prepare_ticks: 0,
            settle_ticks: 0,
            enable_armed: false,
            last_right_switch: 0,
            last_left_switch: 0,
            recovery_armed: false,
            transmit_log_ticks: 0,
        }
    }
}

impl Control {
    /// Clear both motors' voltage commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input stops output; a latched Error is preserved.
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        if self.remote.is_none() && self.state != State::Error {
            self.state = State::Waiting;
        }
        self.update();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the commands; return to Waiting unless already in Error.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            // 断联/无效帧后不能自动使能：清掉“已下拨”记忆，恢复信号后必须重新下拨→中位。
            self.enable_armed = false;
            if self.state != State::Error {
                self.state = State::Waiting;
            }
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        self.yaw.feedback_age_ticks = self.yaw.feedback_age_ticks.saturating_add(1);
        self.pitch.feedback_age_ticks = self.pitch.feedback_age_ticks.saturating_add(1);

        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => self.route_feedback(&envelope.frame),
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    fn route_feedback(&mut self, frame: &Frame) {
        let data = frame.data();
        if data.len() != 8 {
            return;
        }
        let bytes: [u8; 8] = match data.try_into() {
            Ok(bytes) => bytes,
            Err(_) => return,
        };
        let raw_id = match frame.id() {
            Id::Standard(standard) => standard.as_raw(),
            _ => return,
        };
        if raw_id == MOTOR_FEEDBACK_ID_BASE + u16::from(YAW_MOTOR_ID) {
            self.yaw.motor.read(&bytes);
            self.yaw.feedback_age_ticks = 0;
        } else if raw_id == MOTOR_FEEDBACK_ID_BASE + u16::from(PITCH_MOTOR_ID) {
            self.pitch.motor.read(&bytes);
            self.pitch.feedback_age_ticks = 0;
        }
    }

    /// 两轴反馈都新鲜（最近 FEEDBACK_TIMEOUT_TICKS 内收到过有效帧）。
    fn feedback_fresh(&self) -> bool {
        self.yaw.feedback_age_ticks <= FEEDBACK_TIMEOUT_TICKS
            && self.pitch.feedback_age_ticks <= FEEDBACK_TIMEOUT_TICKS
    }

    /// Choose the state from the remote, feedback and fault conditions.
    pub fn update_mode(&mut self) {
        // 反馈超时：非故障状态回 Waiting 并停输出；Error 保留（断联不能清除故障）。
        if self.state != State::Error && !self.feedback_fresh() {
            self.enter_waiting("电机反馈超时");
        }

        let remote = self.remote;
        match self.state {
            State::Waiting => self.update_mode_waiting(remote),
            State::Prepare => self.update_mode_prepare(remote),
            State::Running => self.update_mode_running(remote),
            State::Error => self.update_mode_error(remote),
        }

        // 记录本周期拨杆位置，供下一周期判断“一次新的操作”。
        self.last_right_switch = remote.map_or(0, |r| right_switch(&r));
        self.last_left_switch = remote.map_or(0, |r| left_switch(&r));
    }

    /// Waiting：零输出；先下拨(2)、再回中位(3)，且两轴反馈新鲜，才算一次新的使能。
    fn update_mode_waiting(&mut self, remote: Option<dm_mc02::Dr16>) {
        let Some(remote) = remote else {
            return;
        };
        let right = right_switch(&remote);
        if right == 2 {
            self.enable_armed = true;
        }
        if right == 3 {
            if self.enable_armed && self.feedback_fresh() {
                self.start_prepare();
            } else if self.enable_armed {
                // 反馈没就绪：作废这次使能，避免“一直中位”被当成新的启动。
                self.enable_armed = false;
                rtt_target::rprintln!("[enable] 反馈未就绪，本次使能作废：请先下拨，再拨回中位");
            }
        }
    }

    /// Prepare：回正中；下拨停止、超时退出、两轴稳定后进 Running。
    fn update_mode_prepare(&mut self, remote: Option<dm_mc02::Dr16>) {
        let Some(remote) = remote else {
            return;
        };
        if right_switch(&remote) == 2 {
            self.enter_waiting("Prepare 中下拨 disable");
            return;
        }
        self.prepare_ticks = self.prepare_ticks.saturating_add(1);
        if self.prepare_ticks >= PREPARE_TIMEOUT_TICKS {
            self.enter_waiting("回正超时");
            return;
        }
        if self.recenter_settled() {
            self.settle_ticks = self.settle_ticks.saturating_add(1);
            if self.settle_ticks >= SETTLE_TICKS {
                self.enter_running_from_prepare();
            }
        } else {
            self.settle_ticks = 0;
        }
    }

    /// Running：调试中；下拨取消输出（故障检测在 update 里做）。
    fn update_mode_running(&mut self, remote: Option<dm_mc02::Dr16>) {
        let Some(remote) = remote else {
            return;
        };
        if right_switch(&remote) == 2 {
            self.enter_waiting("下拨 disable");
        }
    }

    /// Error：保持零输出，等待人工恢复。
    /// 恢复动作：右上保持中位(3)，左上先下拨(2)武装、再上拨(1)触发（拨杆会经过中位，用记忆不用相邻边沿）。
    fn update_mode_error(&mut self, remote: Option<dm_mc02::Dr16>) {
        let Some(remote) = remote else {
            // 断联时保持 Error，输出为 0；重连后仍需人工恢复。
            return;
        };

        let right = right_switch(&remote);
        let left = left_switch(&remote);

        // 恢复手势要求右上保持中位；离开中位就作废本次手势。
        if right != 3 {
            self.recovery_armed = false;
            return;
        }
        // 先下拨(2)：武装。注意拨杆从下位拨到上位一定会经过中位，
        // 所以不能用“上一周期恰好是 2”的边沿判断，否则永远等不到。
        if left == 2 {
            self.recovery_armed = true;
        }
        // 再上拨(1)：触发一次恢复；要求反馈新鲜。
        if left == 1 && self.recovery_armed && self.feedback_fresh() {
            self.recover_from_error();
        }
    }

    /// 两轴都进入回正完成判据：误差小、速度小。
    fn recenter_settled(&self) -> bool {
        let yaw_ok = self.yaw.control_error().abs() <= SETTLE_ERR_RAD
            && self.yaw.motor.speed_rpm.abs() <= SETTLE_SPEED_RPM;
        let pitch_ok = self.pitch.control_error().abs() <= SETTLE_ERR_RAD
            && self.pitch.motor.speed_rpm.abs() <= SETTLE_SPEED_RPM;
        yaw_ok && pitch_ok
    }

    /// 参考姿态对应的关节角：已标定 = 0；未标定 = 当前关节角（保证不主动运动）。
    fn reference_target(axis: &Axis) -> f32 {
        if axis.reference_raw.is_some() {
            0.0
        } else {
            axis.joint_angle_rad()
        }
    }

    /// 进入 Prepare：初始化目标与 PID，从参考姿态开始受限回正。
    fn start_prepare(&mut self) {
        let yaw_target = Self::reference_target(&self.yaw);
        let pitch_target = Self::reference_target(&self.pitch);
        self.yaw.reset_control(yaw_target);
        self.pitch.reset_control(pitch_target);

        self.state = State::Prepare;
        self.prepare_ticks = 0;
        self.settle_ticks = 0;
        self.enable_armed = false;
        self.fault = None;

        if self.yaw.reference_raw.is_some() && self.pitch.reference_raw.is_some() {
            rtt_target::rprintln!(
                "[state] Prepare: 回正到参考姿态 yaw={:.3} pitch={:.3} rad",
                self.yaw.target_angle_rad,
                self.pitch.target_angle_rad
            );
        } else {
            rtt_target::rprintln!(
                "[state] Prepare: 参考零位未标定（REFERENCE_RAW=None），不自动回正，保持当前姿态"
            );
        }
    }

    /// Prepare → Running：回正完成。
    fn enter_running_from_prepare(&mut self) {
        self.state = State::Running;
        self.prepare_ticks = 0;
        self.settle_ticks = 0;
        // 目标保持参考姿态；清理积分，避免把回正过程的积分带进调试。
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();
        rtt_target::rprintln!("[state] Running: 回正完成，进入遥控调试");
    }

    /// Error → Running（人工恢复）：目标=当前角，速度目标 0，清 PID 历史，不强制回正。
    fn recover_from_error(&mut self) {
        let yaw_now = self.yaw.joint_angle_rad();
        let pitch_now = self.pitch.joint_angle_rad();
        self.yaw.target_angle_rad = yaw_now;
        self.pitch.target_angle_rad = pitch_now;
        self.yaw.target_rpm = 0.0;
        self.pitch.target_rpm = 0.0;
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();
        self.yaw.reset_detection();
        self.pitch.reset_detection();
        self.fault = None;
        self.recovery_armed = false;
        self.state = State::Running;
        rtt_target::rprintln!(
            "[state] Running: 人工恢复，目标=当前角度 yaw={:.3} pitch={:.3} rad",
            yaw_now,
            pitch_now
        );
    }

    /// 回到 Waiting：清零输出与检测状态，按需打印一次原因。
    fn enter_waiting(&mut self, reason: &str) {
        let was = self.state;
        self.state = State::Waiting;
        self.disable_all();
        self.yaw.reset_detection();
        self.pitch.reset_detection();
        self.prepare_ticks = 0;
        self.settle_ticks = 0;
        if was != State::Waiting {
            rtt_target::rprintln!("[state] Waiting: {}", reason);
        }
    }

    /// 进入 Error：先打印触发瞬间的数值，再清零输出与检测计数，并锁存原因。
    fn enter_error(&mut self, fault: Fault) {
        self.state = State::Error;
        self.fault = Some(fault);
        self.enable_armed = false;
        self.recovery_armed = false;
        match fault {
            Fault::YawStall => {
                rtt_target::rprintln!(
                    "[fault] yaw 疑似堵转: target_rpm={:.0} output={:.0} speed={:.1} err={:.3} stall_ticks={}",
                    self.yaw.target_rpm,
                    self.yaw.motor.command_voltage_raw,
                    self.yaw.motor.speed_rpm,
                    self.yaw.control_error(),
                    self.yaw.stall_ticks
                );
            }
            Fault::PitchStall => {
                rtt_target::rprintln!(
                    "[fault] pitch 疑似堵转: target_rpm={:.0} output={:.0} speed={:.1} err={:.3} stall_ticks={}",
                    self.pitch.target_rpm,
                    self.pitch.motor.command_voltage_raw,
                    self.pitch.motor.speed_rpm,
                    self.pitch.control_error(),
                    self.pitch.stall_ticks
                );
            }
            Fault::YawOscillation => {
                rtt_target::rprintln!(
                    "[fault] yaw 持续高频振荡: {:.1} Hz（连续 {} 个窗口）",
                    self.yaw.osc_freq_hz,
                    OSC_MAX_WINDOWS
                );
            }
            Fault::PitchOscillation => {
                rtt_target::rprintln!(
                    "[fault] pitch 持续高频振荡: {:.1} Hz（连续 {} 个窗口）",
                    self.pitch.osc_freq_hz,
                    OSC_MAX_WINDOWS
                );
            }
        }
        rtt_target::rprintln!(
            "[fault] 已进入 Error，两轴输出清零；恢复动作：右上中位 + 左上先下后上"
        );
        self.disable_all();
        self.yaw.reset_detection();
        self.pitch.reset_detection();
    }

    /// 逐一检查两轴故障判据；任一触发则进入 Error。
    fn detect_faults(&mut self) {
        if let Some(kind) = self.yaw.update_fault_detection() {
            let fault = match kind {
                AxisFault::Stall => Fault::YawStall,
                AxisFault::Oscillation => Fault::YawOscillation,
            };
            self.enter_error(fault);
            return;
        }
        if let Some(kind) = self.pitch.update_fault_detection() {
            let fault = match kind {
                AxisFault::Stall => Fault::PitchStall,
                AxisFault::Oscillation => Fault::PitchOscillation,
            };
            self.enter_error(fault);
        }
    }

    /// Running：左摇杆按“目标角速度”移动目标；松手后目标保持。
    fn update_stick_targets(&mut self) {
        let Some(remote) = self.remote else {
            return;
        };
        // 左摇杆：通道 2 左右（yaw），通道 3 上下（pitch）。
        let yaw_input = apply_deadband(remote.channels[2] as f32 / 660.0);
        let pitch_input = apply_deadband(remote.channels[3] as f32 / 660.0);

        let yaw_target = self.yaw.target_angle_rad + yaw_input * YAW_MAX_TARGET_RATE_RAD_S * 0.001;
        let pitch_target =
            self.pitch.target_angle_rad + pitch_input * PITCH_MAX_TARGET_RATE_RAD_S * 0.001;
        self.yaw.target_angle_rad = self.yaw.sanitize_target(yaw_target);
        self.pitch.target_angle_rad = self.pitch.sanitize_target(pitch_target);
    }

    /// 单轴双环：位置环（rad→rpm）→ 速度环（rpm→电压原始值）。
    /// `max_target_rpm` / `max_output_raw` / `max_output_step_raw` 只在 Prepare 给出，用于限速/限幅/限斜率。
    fn drive_axis(
        axis: &mut Axis,
        dt_s: f32,
        max_target_rpm: Option<f32>,
        max_output_raw: Option<f32>,
        max_output_step_raw: Option<f32>,
    ) {
        // 位置环误差在关节坐标里算（已处理回绕/限位），再交给 PID。
        let error = axis.control_error();
        let measured_equivalent = axis.target_angle_rad - error;
        let mut target_rpm =
            axis.position_pid
                .calculate(axis.target_angle_rad, measured_equivalent, dt_s);
        if let Some(limit) = max_target_rpm {
            target_rpm = clamp_symmetric(target_rpm, limit);
        }

        // 方向校准：关节坐标的正输出乘安装方向，才是电机轴该转的方向（只乘一次）。
        axis.target_rpm = axis.direction * target_rpm;

        let mut output = axis
            .speed_pid
            .calculate(axis.target_rpm, axis.motor.speed_rpm, dt_s);
        if let Some(limit) = max_output_raw {
            output = clamp_symmetric(output, limit);
        }
        if let Some(step) = max_output_step_raw {
            let delta = clamp_symmetric(output - axis.previous_command_raw, step);
            output = axis.previous_command_raw + delta;
        }
        axis.previous_command_raw = output;
        axis.motor.command_voltage_raw = output;
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    pub fn update(&mut self) {
        match self.state {
            // 未使能 / 故障：两轴零输出。
            State::Waiting | State::Error => {
                self.disable_all();
                self.yaw.reset_detection();
                self.pitch.reset_detection();
            }
            // 回正：目标已在进入 Prepare 时固定为参考姿态，这里只做受限跟踪。
            State::Prepare => {
                Self::drive_axis(
                    &mut self.yaw,
                    0.001,
                    Some(PREPARE_MAX_TARGET_RPM),
                    Some(PREPARE_MAX_OUTPUT_RAW),
                    Some(PREPARE_MAX_OUTPUT_STEP_RAW),
                );
                Self::drive_axis(
                    &mut self.pitch,
                    0.001,
                    Some(PREPARE_MAX_TARGET_RPM),
                    Some(PREPARE_MAX_OUTPUT_RAW),
                    Some(PREPARE_MAX_OUTPUT_STEP_RAW),
                );
                // 起步 500 ms 只加速、不判故障。
                if self.prepare_ticks > PREPARE_FAULT_GRACE_TICKS {
                    self.detect_faults();
                }
            }
            // 调试：摇杆移动目标，双环跟踪。
            State::Running => {
                self.update_stick_targets();
                Self::drive_axis(&mut self.yaw, 0.001, None, None, Some(RUNNING_MAX_OUTPUT_STEP_RAW));
                Self::drive_axis(&mut self.pitch, 0.001, None, None, Some(RUNNING_MAX_OUTPUT_STEP_RAW));
                self.detect_faults();
            }
        }
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];

        let output_allowed = matches!(self.state, State::Prepare | State::Running)
            && self.yaw.feedback_age_ticks <= FEEDBACK_TIMEOUT_TICKS
            && self.pitch.feedback_age_ticks <= FEEDBACK_TIMEOUT_TICKS;
        if output_allowed {
            encode_voltage_command(&mut data, YAW_MOTOR_ID, self.yaw.motor.command_voltage_raw);
            encode_voltage_command(
                &mut data,
                PITCH_MOTOR_ID,
                self.pitch.motor.command_voltage_raw,
            );
        }

        // 限频打印编码结果（约 1 s 一次）：核对帧 ID、槽位顺序、“零输出=八个 0”。
        self.transmit_log_ticks = self.transmit_log_ticks.wrapping_add(1);
        if self.transmit_log_ticks >= TX_LOG_PERIOD_TICKS {
            self.transmit_log_ticks = 0;
            rtt_target::rprintln!(
                "TX id=0x{:03X} data=[{:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}]",
                MOTOR_COMMAND_ID,
                data[0],
                data[1],
                data[2],
                data[3],
                data[4],
                data[5],
                data[6],
                data[7],
            );
        }

        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

// Set up RTT and the board, then start the remote and control tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!(
        "Gimbal control started (yaw motor ID {}, pitch motor ID {}, command 0x1FF)",
        YAW_MOTOR_ID,
        PITCH_MOTOR_ID
    );
    if YAW_REFERENCE_RAW.is_none() || PITCH_REFERENCE_RAW.is_none() {
        rtt_target::rprintln!(
            "注意：参考零位未标定，Prepare 不会自动回正；现场按 README 3.2 填 YAW_REFERENCE_RAW/PITCH_REFERENCE_RAW"
        );
    }
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    // 约每 200 ms 打印一次关键量；1 kHz 循环里不要每周期打印。
    let mut log_ticks: u16 = 0;
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        watchdog.pet();

        log_ticks = log_ticks.wrapping_add(1);
        if log_ticks >= 200 {
            log_ticks = 0;
            rtt_target::rprintln!(
                "state={:?} fault={:?} tx_failed={} prep={} settle={} | yaw j={:.3} raw={:.3} tgt={:.3} \
                 rpm={:.1} trpm={:.1} out={:.0} cur={} stall={} osc={:.1}Hz | pitch j={:.3} raw={:.3} tgt={:.3} \
                 rpm={:.1} trpm={:.1} out={:.0} cur={} stall={} osc={:.1}Hz",
                control.state,
                control.fault,
                TX_FAILED.load(Ordering::Relaxed),
                control.prepare_ticks,
                control.settle_ticks,
                control.yaw.joint_angle_rad(),
                control.yaw.motor.angle_rad,
                control.yaw.target_angle_rad,
                control.yaw.motor.speed_rpm,
                control.yaw.target_rpm,
                control.yaw.motor.command_voltage_raw,
                control.yaw.motor.current_raw,
                control.yaw.stall_ticks,
                control.yaw.osc_freq_hz,
                control.pitch.joint_angle_rad(),
                control.pitch.motor.angle_rad,
                control.pitch.target_angle_rad,
                control.pitch.motor.speed_rpm,
                control.pitch.target_rpm,
                control.pitch.motor.command_voltage_raw,
                control.pitch.motor.current_raw,
                control.pitch.stall_ticks,
                control.pitch.osc_freq_hz,
            );
        }
    }
}

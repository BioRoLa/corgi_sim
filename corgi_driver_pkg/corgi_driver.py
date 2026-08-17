import rclpy
import math
import os
import yaml

# --- Webots 控制器模組 (用於控制模擬狀態) ---
from controller import Supervisor

# --- [新增] Supervisor 與 TF 相關模組 ---
from rosgraph_msgs.msg import Clock
from corgi_msgs.msg import TriggerStamped
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from tf2_ros import TransformBroadcaster
from corgi_msgs.msg import MotorCmdStamped
from corgi_msgs.msg import MotorStateStamped, MotorState
from corgi_msgs.msg import ImuStamped
from corgi_msgs.msg import RobotStateStamped
from corgi_msgs.msg import SimLegContactStamped, SimLegContact

from . import Controller_TB
from .LegModel import LegModel
from .motor_config import LEG_CONFIG


# ABAD joint range, radians. The real mechanism stops at about +/-70 deg; the
# proto now enforces it (minPosition/maxPosition on each *_ABAD motor) and this
# is the matching software guard, so an out-of-range command is reported rather
# than silently absorbed. Override with CORGI_ABAD_LIMIT_DEG.
ABAD_LIMIT_RAD = math.radians(float(os.environ.get("CORGI_ABAD_LIMIT_DEG", "70")))


def _load_leg_config():
    """從與本模組相同目錄的 motor_config.yaml 讀取關節方向設定。
    修改 YAML 後直接重啟 Webots 即可，不需要 colcon build。"""
    yaml_path = os.path.join(os.path.dirname(__file__), 'motor_config.yaml')
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)


LEG_CONFIG = _load_leg_config()

class imu:
    def __init__(self, robot, node, basic_time_step=1):
        """imu Group for Robots

        Args:
            robot (_type_): webots robot
            node (_type_): ROS2 node
            basic_time_step (int, optional): _description_. Defaults to 1.
        """
        # sensors: imu gyro ang_vel
        
        # Inertial Measurement Unit
        # Accelerometer in Webots
        # returns acceleration
        try:
            self.sensor_Accelerometer = robot.getDevice("imu")
            self.sensor_Accelerometer.enable(basic_time_step)
        except:
            self.sensor_Accelerometer = None
            print("No IMU Sensor Found")
        
        # Gyro Sensor
        # Gyro in Webots
        # returns angular velocity
        try:
            self.sensor_Gyro = robot.getDevice("ang_vel")
            self.sensor_Gyro.enable(basic_time_step)
        except:
            self.sensor_Gyro = None
            print("No Gyro Sensor Found")
        
        # Accelerometer Sensor
        # InertialUnit in Webots
        # returns quaternion
        try:
            self.sensor_InertialUnit = robot.getDevice("gyro")
            self.sensor_InertialUnit.enable(basic_time_step)
        except:
            self.sensor_InertialUnit = None
            print("No Accelerometer Sensor Found")
        
        # Node setup
        self.__node = node
        # imu publisher
        self.imu_pub = node.create_publisher(
            ImuStamped,
            'imu',
            1000
        )
        
    def get_msg(self, time_stamp = Time(), seq = -1):
        """
        Get imu message
        Args:
            time_stamp (Time, optional): _description_. Defaults to Time().
            seq (int, optional): _description_. Defaults to -1.
        Returns:
            imuStamped: _description_
        """
        # imustamp:
        # Headers header: self-defined header
        # geometry_msgs/Quaternion orientation
        # geometry_msgs/Vector3 angular_velocity
        # geometry_msgs/Vector3 linear_acceleration
        
        msg = ImuStamped()
        msg.header.stamp = time_stamp
        msg.header.seq = seq
        
        if self.sensor_InertialUnit:
            quat = self.sensor_InertialUnit.getQuaternion()
            msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        if self.sensor_Gyro:
            gyro = self.sensor_Gyro.getValues()
            msg.angular_velocity = Vector3(x=gyro[0], y=gyro[1], z=gyro[2])
        if self.sensor_Accelerometer:
            acc = self.sensor_Accelerometer.getValues()
            msg.linear_acceleration = Vector3(x=acc[0], y=acc[1], z=acc[2])
        return msg

# --- Torque ceilings ---------------------------------------------------------
#
# PROVENANCE (record it here; the previous value had none). Motor is the Haitai
# HT-04 / HT8115-J6 on BOTH the leg and the ABAD, run at 48 V. Datasheet, after
# the stock 6:1 planetary: rated 11 N.m, stall 29.5 N.m, stall current 45 A,
# torque constant 0.7 N.m/A referred to the output (11/16 = 0.69), max speed
# 330 rpm at 48 V.
#
# The gearboxes differ: LEG 6:1 (stock, so the datasheet figures apply as
# printed) and ABAD 9:1 (torque x1.5, speed /1.5). Assumes equal gearbox
# efficiency -- check that before leaning on the ABAD figure for a claim.
#
# These are STALL torques, i.e. the absolute ceiling at zero speed. Available
# torque droops with speed: the leg runs ~40-60 rpm during stance, ~15% of its
# 330 rpm no-load, so its realistic ceiling is nearer 25 N.m. Model that with
# CORGI_MAX_TORQUE_LEG if a run needs to be conservative.
#
# The previous shared value of 35.0 was ABOVE the leg's stall torque -- a torque
# the hardware cannot produce at any speed -- and BELOW the ABAD's.
MAX_TORQUE_LEG = 29.5     # N.m, HT-04 stall @ 6:1
MAX_TORQUE_ABAD = 44.25   # N.m, HT-04 stall @ 9:1 (29.5 * 9/6)


def resolve_torque_limits():
    """(leg, abad) ceilings in N.m, with env overrides.

    CORGI_MAX_TORQUE_LEG / _ABAD set one joint each. CORGI_MAX_TORQUE sets BOTH
    -- kept because sweep_torque_ceiling.sh drives it to make the clipped demand
    observable, and that experiment wants one knob, not two.

    Raising any of these does NOT model a stronger motor. It is an instrument
    for seeing demand that would otherwise be clipped, and results taken with it
    raised must not be read as "the robot would do this with better hardware".
    """
    both = os.environ.get("CORGI_MAX_TORQUE")
    leg = float(os.environ.get("CORGI_MAX_TORQUE_LEG", both or MAX_TORQUE_LEG))
    abad = float(os.environ.get("CORGI_MAX_TORQUE_ABAD", both or MAX_TORQUE_ABAD))
    return leg, abad


class LegManager:
    def __init__(self, robot, prefix, controller_tb, basic_time_step=1,
                 max_torque_leg=None, max_torque_abad=None,
                 abad_prefix=None, leg_config=None):
        self.prefix = prefix
        self._abad_clamp_warned = False
        self.motors = {}
        self.sensors = {}
        self.tb = controller_tb

        cfg = leg_config or {}
        jdir = cfg.get('joint_dir', {})
        mdir = cfg.get('motor_dir', {})
        self.dir_theta    = jdir.get('theta',        1.0)
        self.dir_beta     = jdir.get('beta',         1.0)
        self.dir_g_beta   = jdir.get('g_joint_beta', 1.0)
        self.dir_motor_l  = mdir.get('L',    1.0)
        self.dir_motor_r  = mdir.get('R',    1.0)
        self.dir_abad     = mdir.get('ABAD', 1.0)
        motor_names = ["L_Motor", "R_Motor"]
        self.prev_pos_l = None
        self.prev_pos_r = None
        self.prev_pos_h = None
        self.prev_vel_l = 0.0
        self.prev_vel_r = 0.0
        self.prev_vel_h = 0.0
        # 當前速度（用於狀態發布，避免重複計算）
        self.current_vel_l = 0.0
        self.current_vel_r = 0.0
        self.current_vel_h = 0.0
        # 扭矩命令存儲（用於發布命令值）
        self.cmd_trq_l = 0.0
        self.cmd_trq_r = 0.0
        self.cmd_trq_h = 0.0
        self.basic_time_step = basic_time_step
        # Per-joint torque ceilings. The leg and ABAD run the SAME motor
        # (Haitai HT-04 / HT8115-J6, 48 V) but DIFFERENT gearboxes -- leg
        # 6:1, ABAD 9:1 -- so their output ceilings differ by 1.5x. A
        # single shared clamp is wrong for both at once: it was 19% too
        # generous on the leg and 21% too tight on the ABAD.
        self.max_torque_leg = (MAX_TORQUE_LEG if max_torque_leg is None
                               else float(max_torque_leg))
        self.max_torque_abad = (MAX_TORQUE_ABAD if max_torque_abad is None
                                else float(max_torque_abad))
        self.max_torque = {
            "L_Motor": self.max_torque_leg,
            "R_Motor": self.max_torque_leg,
            "ABAD":    self.max_torque_abad,
        }
        
        for name in motor_names:
            full_name = prefix + name
            sensor_full_name = full_name + "_sensor"
            sensor = robot.getDevice(sensor_full_name)
            if sensor:
                self.sensors[name] = sensor
                sensor.enable(self.basic_time_step)
            motor = robot.getDevice(full_name)
            if motor:
                self.motors[name] = motor
                # 設定為扭矩控制模式
                motor.setPosition(float('inf'))  # 無限位置 = 不使用位置控制
                motor.setVelocity(0.0)           # 初始速度為 0
                motor.enableTorqueFeedback(self.basic_time_step)
                motor.setAvailableTorque(self.max_torque_leg)
        
        # --- ABAD 馬達 (力矩控制) ---
        self.motor_abad = None
        self.sensor_abad = None
        if abad_prefix:
            abad_name = f"{abad_prefix}_ABAD"
            abad_sensor_name = f"{abad_name}_sensor"

            self.sensor_abad = robot.getDevice(abad_sensor_name)
            if self.sensor_abad:
                self.sensors["ABAD"] = self.sensor_abad
                self.sensor_abad.enable(self.basic_time_step)
            else:
                print(f"Warning: 找不到 ABAD 感測器 {abad_sensor_name}")

            self.motor_abad = robot.getDevice(abad_name)
            if self.motor_abad:
                self.motors["ABAD"] = self.motor_abad
                self.motor_abad.setPosition(float('inf'))
                self.motor_abad.setVelocity(0.0)
                self.motor_abad.enableTorqueFeedback(self.basic_time_step)
                self.motor_abad.setAvailableTorque(self.max_torque_abad)
            else:
                print(f"Warning: 找不到 ABAD 馬達 {abad_name}")
        
        # --- G_Joint 被動追蹤 (透過 LegModel) ---
        self.motor_g_joint = None
        self.leg_model = None
        self.G_Offset = None
        try:
            g_joint_name = prefix + "G_Joint"
            g_joint = robot.getDevice(g_joint_name)
            if g_joint:
                self.motor_g_joint = g_joint
                self.leg_model = LegModel(sim=True)
                self.leg_model.forward(math.radians(17), 0.0)
                self.G_Offset = self.leg_model['ang_OGF']
        except Exception:
            self.motor_g_joint = None

    # --- ROS <-> module frame: ONE boundary, matched pairs -------------------
    #
    # joint_dir flips the sense of theta/beta between the ROS convention and
    # this module's own frame. Historically that flip was applied to the
    # POSITION command and to the reported STATE, but not to the feedforward
    # TORQUE -- so legs with dir_beta = -1 (B and C) were commanded a mirrored
    # pose while being fed un-mirrored torques. Statically that is invisible
    # (feedforward ~ 0, the kp term dominates); under load it made B and C
    # reach only 63-70% of their commanded beta sweep against A and D's 92-95%,
    # with a ~28 ms phase lag, which yawed the robot ~15-20 deg per speed rung.
    #
    # These three functions are the whole conversion. Add signs HERE, never at
    # a call site.

    def ros_to_module(self, theta, beta):
        """ROS joint angles -> this module's frame. Self-inverse."""
        return theta * self.dir_theta, beta * self.dir_beta

    def module_to_ros(self, theta, beta):
        """This module's frame -> ROS joint angles. Self-inverse."""
        return theta * self.dir_theta, beta * self.dir_beta

    def convert_torque(self, tau_r, tau_l):
        """Convert motor torques between ROS and module frame. Self-inverse.

        NOT a free convention -- forced by ros_to_module. force_control works
        entirely in ROS-convention phi space:
            fc0 = beta_ros + theta - 17deg      fc1 = beta_ros - theta + 17deg
        while the driver drives IK(theta, beta * dir_beta), so for
        dir_beta = -1 the actual motor angles are
            m0 = -beta_ros + theta - 17 = -fc1      m1 = -fc0
        i.e. SWAP AND NEGATE. Equivalently via virtual work: generalized
        forces transform contravariantly, so flipping beta flips tau_beta and
        leaves tau_theta alone.

        Note the swap is between the (r, l) LABELLED PAIR, which is what makes
        this safe despite force_control indexing fc0 as "l" while the driver's
        coupling calls beta+theta-17 the R motor -- both files are internally
        consistent, and the pair swap is the same operation either way.
        """
        self._require_supported_dirs()
        if self.dir_beta < 0:
            tau_r, tau_l = -tau_l, -tau_r
        return tau_r, tau_l

    def convert_gains(self, g_r, g_l):
        """Convert a PD gain pair between ROS and module frame. Self-inverse.

        The gains need the SAME swap as the torques but WITHOUT the negation,
        and missing this is what made the torque-only fix over-correct.

        The module-frame error flips sign along with the angle
        (err_m0 = -err_fc1), so
            tau_m0 = -tau_fc1 = -kp_fc1 * err_fc1 = kp_fc1 * err_m0
        -- the gain that belongs on module motor 0 is the OTHER motor's gain,
        un-negated. A gain is a positive scalar; negating it would invert the
        servo.
        """
        self._require_supported_dirs()
        if self.dir_beta < 0:
            g_r, g_l = g_l, g_r
        return g_r, g_l

    def _require_supported_dirs(self):
        """dir_theta < 0 is not supported and is not used by any leg.

        A theta flip does NOT reduce to a clean swap, because the coupling
        carries a constant +-17 deg offset: m0 = beta - theta - 17 is not
        +-fc0 or +-fc1. It would need its own derivation. All four legs ship
        dir_theta = +1, so fail loudly rather than apply an unverified map.
        """
        if self.dir_theta < 0:
            raise NotImplementedError(
                f"[{self.prefix}] dir_theta < 0 is not supported: the 17 deg "
                "coupling offset breaks the swap mapping; re-derive first.")

    def _apply_torque_control(self, motor_name, cmd_pos, current_pos, current_vel,
                              kp=0.0, kd=0.0, torque_ff=0.0):
        """使用 PD + feedforward 控制律計算並套用扭矩。"""
        pos_error = cmd_pos - current_pos
        torque_cmd = kp * pos_error + kd * (-current_vel) + torque_ff
        # motor_name is one of L_Motor / R_Motor / ABAD; fall back to the
        # leg (lower) ceiling rather than the ABAD one if it is ever
        # something else, so an unknown joint fails safe.
        lim = self.max_torque.get(motor_name, self.max_torque_leg)
        torque_cmd = max(-lim, min(lim, torque_cmd))

        if motor_name in self.motors:
            self.motors[motor_name].setTorque(torque_cmd)

        return torque_cmd, pos_error

    def set_target(self, theta, beta, kp_r=0.0, kp_l=0.0, kd_r=0.0, kd_l=0.0, torque_r=0.0, torque_l=0.0):
        """
        設定腿部目標 - 純扭矩控制
        
        Args:
            theta: 腿部伸展角 (rad)
            beta: 腿部旋轉角 (rad)
            kp_r, kp_l: 位置比例增益
            kd_r, kd_l: 速度阻尼增益
            torque_r, torque_l: 前饋扭矩 (N·m)
        
        控制律:
            τ = kp × (φ_desired - φ_actual) + kd × (-φ̇_actual) + τ_feedforward
        """
        # 限制最小角度
        theta_0 = math.radians(17)
        if theta < theta_0:
            theta = theta_0
        
        # 計算目標馬達角度 (IK)
        cmd_L, cmd_R = self.tb.IK(theta, beta)
        
        # 讀取當前馬達狀態
        pos_r = self.sensors["R_Motor"].getValue()
        pos_l = self.sensors["L_Motor"].getValue()
        alpha = 1  # 低通濾波係數
        
        # 初始化前一時刻的位置
        if self.prev_pos_r is None:
            self.prev_pos_r = pos_r
        if self.prev_pos_l is None:
            self.prev_pos_l = pos_l
        
        # 計算速度 (rad/s) - 使用數值微分
        dt = self.basic_time_step / 1000.0  # 轉換為秒
        vel_r = (pos_r - self.prev_pos_r) / dt
        vel_l = (pos_l - self.prev_pos_l) / dt
        # 速度低通濾波
        vel_r = alpha * vel_r + (1 - alpha) * self.prev_vel_r
        vel_l = alpha * vel_l + (1 - alpha) * self.prev_vel_l
        # 更新歷史位置和速度
        self.prev_pos_r = pos_r
        self.prev_pos_l = pos_l
        self.prev_vel_r = vel_r
        self.prev_vel_l = vel_l
        # 保存當前速度供get_states使用，避免重複計算
        self.current_vel_l = vel_l
        self.current_vel_r = vel_r
        
        # 處理角度連續性（避免 ±π 跳變）
        cmd_R = self._find_closest_phi(cmd_R, pos_r)
        cmd_L = self._find_closest_phi(cmd_L, pos_l)
        
        # trq = kp * (phi_desired - phi_actual) + kd * (-phi_dot_actual) + torque_ff
        trq_r, err_r = self._apply_torque_control(
            "R_Motor", cmd_R, pos_r * self.dir_motor_r, vel_r * self.dir_motor_r,
            kp=kp_r, kd=kd_r, torque_ff=torque_r * self.dir_motor_r
        )
        trq_l, err_l = self._apply_torque_control(
            "L_Motor", cmd_L, pos_l * self.dir_motor_l, vel_l * self.dir_motor_l,
            kp=kp_l, kd=kd_l, torque_ff=torque_l * self.dir_motor_l
        )
        
        # 保存實際套用的扭矩命令
        self.cmd_trq_r = trq_r
        self.cmd_trq_l = trq_l
        
        # print debug info
        return "".join([f"[{self.prefix}] Target θ: {theta:.3f} rad, β: {beta:.3f} rad | ",
                                      f"Cmd L: {cmd_L:.3f} rad, R: {cmd_R:.3f} rad | ",
                                      f"Pos L: {pos_l:.3f} rad, R: {pos_r:.3f} rad | ",
                                      f"Err L: {err_l:.3f} rad, R: {err_r:.3f} rad | ",
                                      f"Vel L: {vel_l:.3f} rad/s, R: {vel_r:.3f} rad/s | ",
                                      f"Trq L: {trq_l:.3f} Nm, R: {trq_r:.3f} Nm"])
    
    def set_abad(self, gamma, kp=0.0, kd=0.0, torque=0.0):
        """設定 ABAD 馬達目標角度 (力矩控制)
        
        Args:
            gamma: ABAD 角度 (rad)
            kp: 位置比例增益
            kd: 速度阻尼增益
            torque: 前饋扭矩 (N·m)
        """
        if not self.motor_abad or not self.sensor_abad:
            return None

        pos_h = self.sensor_abad.getValue()
        if self.prev_pos_h is None:
            self.prev_pos_h = pos_h

        dt = self.basic_time_step / 1000.0
        alpha = 1
        vel_h = (pos_h - self.prev_pos_h) / dt
        vel_h = alpha * vel_h + (1 - alpha) * self.prev_vel_h

        self.prev_pos_h = pos_h
        self.prev_vel_h = vel_h

        # The ABAD is a LIMITED joint (+/-70 deg, now enforced in the proto), so
        # it must NOT be unwrapped the way the continuously-rotating hip motors
        # are. _find_closest_phi picks whichever 2*pi-equivalent sits nearest the
        # current position, which on a limited joint means a wound-up ABAD gets
        # FOLLOWED round instead of pulled back -- exactly what let the joints
        # spin through full circles on 2026-08-14 while the logged angles still
        # looked plausible. Command the angle as given, and clamp.
        gamma_target = gamma * self.dir_abad
        if abs(gamma_target) > ABAD_LIMIT_RAD:
            # Print once per leg rather than at 1 kHz. The point is to make an
            # impossible command VISIBLE -- silently clamping is how a pose the
            # hardware cannot reach ends up in a result table.
            if not self._abad_clamp_warned:
                self._abad_clamp_warned = True
                print(f"[{self.prefix}] ABAD command "
                      f"{math.degrees(gamma_target):+.1f} deg is outside the "
                      f"+/-{math.degrees(ABAD_LIMIT_RAD):.0f} deg range; "
                      f"clamping. The hardware cannot reach this pose.",
                      flush=True)
            gamma_target = math.copysign(ABAD_LIMIT_RAD, gamma_target)
        trq_h, _ = self._apply_torque_control(
            "ABAD", gamma_target, pos_h, vel_h,
            kp=kp, kd=kd, torque_ff=torque * self.dir_abad
        )

        self.current_vel_h = vel_h * self.dir_abad
        self.cmd_trq_h = trq_h * self.dir_abad
    
    def update_g_joint(self, theta, beta):
        """根據 theta/beta 更新 G_Joint 被動追蹤
        
        Args:
            theta: 腿部伸展角 (rad)
            beta: 腿部旋轉角 (rad)
        """
        if self.motor_g_joint and self.leg_model:
            self.leg_model.theta = theta
            self.leg_model.beta = beta * self.dir_g_beta
            self.leg_model.calculate()
            g_joint_target = self.leg_model['ang_OGF'] - self.G_Offset
            self.motor_g_joint.setPosition(g_joint_target)
    
    def _find_closest_phi(self, phi_target, phi_current):
        """
        找到最接近的等價角度（處理 2π 週期性）
        """
        diff = (phi_target - phi_current + math.pi) % (2 * math.pi) - math.pi
        return phi_current + diff
      
    def get_states(self):
        pos_l = self.sensors["L_Motor"].getValue()
        pos_r = self.sensors["R_Motor"].getValue()
        pos_h = self.sensor_abad.getValue() * self.dir_abad if self.sensor_abad else 0.0
        
        msg = MotorState()
        theta, beta = self.module_to_ros(*self.tb.FK(pos_l, pos_r))
        msg.theta = float(theta)
        msg.beta = float(beta)
        msg.gamma = float(pos_h)
        
        # 直接使用控制器中計算的速度（已經過濾波）
        msg.velocity_l = float(self.current_vel_l)
        msg.velocity_r = float(self.current_vel_r)
        msg.velocity_h = float(self.current_vel_h)
        
        # 發布扭矩命令值（命令扭矩，而非回饋）
        #
        # Reported raw, matching the command path, which does not transform
        # either (see the note in execute_motor). If the transform is ever
        # adopted, BOTH ends must change together.
        msg.torque_r = float(self.cmd_trq_r)
        msg.torque_l = float(self.cmd_trq_l)
        msg.torque_h = float(self.cmd_trq_h)
        return msg
    
class CorgiDriver:
    def init(self, webots_node, properties):

        # 1. set webot
        # get webot robot
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # paused flag
        self.has_paused = False

        # 2. init ros2 node
        # 先檢查是否已經 init 過，避免重複報錯
        if not rclpy.ok():
            rclpy.init(args=None)
            
        # 建立名為 corgi_driver 的節點
        self.__node = rclpy.create_node('corgi_driver_internal')
        
        # 取得自己在 Webots 中的節點 (需要 World 裡 supervisor=TRUE)
        self.__self_node = self.__robot.getSelf()
        
        # 建立 /clock 發布器
        self.ros_time_msg = Time()
        self.clock_pub = self.__node.create_publisher(Clock, 'clock', 1000)
        
        # 建立 TF 廣播器 (讓 Rviz 知道機器人在哪)
        self.tf_broadcaster = TransformBroadcaster(self.__node) 
        
        # Fixed PID parameters (not using ROS2 parameter)
        # TUNED Params — updated from PID sweep 2026-06-29
        self.KP = 120.0      # leg (theta/beta) proportional gain
        self.KI = 0.0
        self.KD = 1.75       # leg derivative gain
        self.GAMMA_KP = 150.0  # abad gamma proportional gain
        self.GAMMA_KD = 1.75   # abad gamma derivative gain
        self.max_torque_leg, self.max_torque_abad = resolve_torque_limits()
        self.trq_feedforward = 0  # N·m 前饋扭矩
        
        # 3. initialize Legs (direction config loaded from motor_config.py)
        self.legs = {
            leg_id: LegManager(
                self.__robot,
                f"{leg_id}_Module_",
                Controller_TB.Controller_TB(theta_0=math.radians(17)),
                basic_time_step=self.__timestep,
                max_torque_leg=self.max_torque_leg,
                max_torque_abad=self.max_torque_abad,
                abad_prefix=leg_id,
                leg_config=LEG_CONFIG[leg_id],
            )
            for leg_id in ('A', 'B', 'C', 'D')
        }
        # Announce the ceilings. NOTE: this does NOT reach ramp_ctl.log --
        # the Webots driver's output is not captured by that harness -- so do
        # not rely on it to tell you which limits a run used.
        #
        # The trustworthy check is the DUMP ITSELF: a clamped run's max |tau|
        # equals the ceiling exactly (29.500 / 44.250 here). Stamping the env
        # values into the dump instead would be worse than useless -- that is
        # precisely the S28 failure, where the variable was set and the driver
        # ignored it, so the metadata would have recorded a lie.
        self.__node.get_logger().info(
            f"Torque ceilings: leg {self.max_torque_leg:.2f} N.m (6:1), "
            f"ABAD {self.max_torque_abad:.2f} N.m (9:1) "
            f"[HT-04 stall @ 48 V; override CORGI_MAX_TORQUE_LEG/_ABAD]")

        # 4. initialize IMU
        self.imu_sensor = imu(self.__robot, self.__node, basic_time_step=self.__timestep)
        
        # 5. Motor Command Subscriber
        self.motor_sub = self.__node.create_subscription(
            MotorCmdStamped,
            'motor/command',
            self.cb_motor,
            1
        )
        # Latest received command
        self.latest_command = None
        
        
        # Default position when no message received
        self.default_theta = 0.0
        self.default_beta = 0.0
        self.default_gamma = 0.0
        
        # ROS Control Mode Flag
        # motor state publisher
        self.motor_state_pub = self.__node.create_publisher(
            MotorStateStamped,
            'motor/state',
            1
        )
        
        # FSM publisher
        self.fsm_pub = self.__node.create_publisher(
            RobotStateStamped,
            'robot/state',
            1
        )

        # Contact state publisher
        self.contact_pub = self.__node.create_publisher(
            SimLegContactStamped,
            'sim/leg_contact',
            1000
        )
        # Body odometry publisher (Supervisor ground truth).
        #
        # Nothing downstream could measure body speed: the driver published
        # imu, clock, motor/state, robot/state and sim/leg_contact, and
        # ImuStamped carries only linear_acceleration -- integrating that over
        # a 9.5 s speed ramp drifts too far to trust. So flight fraction was
        # the only available proxy for "is the robot running at the speed the
        # template was solved for", and the ramp's actual speed profile was
        # unverified.
        #
        # pose is in the odom (world) frame; twist is in base_link, per the
        # nav_msgs/Odometry convention. Prefer differencing pose.position.x
        # for a headline speed -- it is the least-processed quantity and is
        # immune to any body-frame rotation mistake.
        self.base_odom_pub = self.__node.create_publisher(
            Odometry,
            'sim/base_odom',
            1000
        )
        # Contact detection: getContactPoints(includeDescendants=True) every 100 steps,
        # result assigned via nearest-module (not quadrant) to avoid cross-boundary bleed.
        self._contact_cache = set()
        # Contact points are refreshed every N simulation steps. At 100 (1 ms
        # step => every 100 ms) a 267 ms hop stride got only ~2.7 contact
        # updates, and its 157 ms flight phase about 1.6 samples -- so whether
        # a flight phase registered at all was close to chance. Measured flight
        # fraction swung between 0% and 36% on identical controller settings,
        # which was mistaken for gait instability.
        #
        # 10 gives ~27 samples per stride, enough to resolve a 43/57 duty
        # split. The refresh calls getContactPoints() on the supervisor, but
        # the per-call loop is only ~12 points; the simulation is dominated by
        # the 300k-triangle mesh physics, not by this.
        self._contact_update_interval = int(
            os.environ.get('CORGI_CONTACT_INTERVAL', '10'))
        # Odometry is sampled on the same interval as contact so the two line
        # up in time -- reading speed against flight phase per stride is the
        # whole point of publishing it.
        self._odom_update_interval = int(
            os.environ.get('CORGI_ODOM_INTERVAL',
                           str(self._contact_update_interval)))
        # Module hip positions in robot body frame: A front-left, B front-right, C rear-right, D rear-left
        self._MODULE_XY = {'A': (0.255, 0.12), 'B': (0.255, -0.12),
                           'C': (-0.255, -0.12), 'D': (-0.255, 0.12)}

        # Foot-frame diagnostic (see log_foot_frame). PROTO-internal DEF nodes
        # are reachable from the PROTO's own controller via getFromProtoDef;
        # getFromDef is the fallback for a non-opaque scene tree.
        # DO NOT use the *_FOOT DEFs here. They label DIFFERENT BODIES on
        # different legs: A_FOOT is a real mesh (DEF F_R) partway up the leg,
        # while B/C/D_FOOT are empty reference links
        # ("Empty_Link_Joint_G_Ref_<X>_Module") 81 mm lower down. Comparing
        # them across legs invents a ~40 mm fore-aft offset and a ~30% swing
        # sensitivity difference on leg A that do not exist -- both were chased
        # as a hardware fault before the proto was read.
        #
        # *_WHEEL_R / *_WHEEL_L exist consistently on all four legs, so take
        # their midpoint: comparable by construction, and close to the contact.
        self._foot_debug = os.environ.get('CORGI_FOOT_DEBUG', '0') == '1'
        self._foot_nodes = {}
        if self._foot_debug:
            for leg_id in ('A', 'B', 'C', 'D'):
                pair = []
                for side in ('R', 'L'):
                    node = None
                    for getter in (
                        lambda n: self.__self_node.getFromProtoDef(n),
                        lambda n: self.__robot.getFromDef(n),
                    ):
                        try:
                            node = getter(f'{leg_id}_WHEEL_{side}')
                        except Exception:
                            node = None
                        if node is not None:
                            break
                    pair.append(node)
                self._foot_nodes[leg_id] = pair if all(pair) else None
            found = [k for k, v in self._foot_nodes.items() if v is not None]
            self.__node.get_logger().info(
                f"[FootFrame] debug on; resolved wheel pairs: {found}")

        # Initialize loop counter
        self.loop_counter = 0

        # --- Experiment Mode ---
        self.experiment_mode = os.environ.get('CORGI_EXPERIMENT_MODE', '0') == '1'
        self.support_box_removed = False
        self._support_box_removed_step = -1  # skip contact for 1 step after removal

        if self.experiment_mode:
            self.__node.get_logger().info("[Experiment Mode] Skipping initial pause. Subscribing to /trigger for support box removal.")
            # Subscribe to /trigger to remove the support box when experiment starts
            self.trigger_sub = self.__node.create_subscription(
                TriggerStamped,
                'trigger',
                self._experiment_trigger_cb,
                10
            )
        else:
            # Normal mode: pause simulation at the beginning
            self.__robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        
        container_id = os.environ.get('HOSTNAME', 'Unknown')
        domain_id = os.environ.get('ROS_DOMAIN_ID', '???')
        self.__node.get_logger().info(f"Driver Initialized! Connected from Container: {container_id} (Domain: {domain_id})")
        
    def _experiment_trigger_cb(self, msg):
        """[Experiment Mode] Remove support box when trigger is enabled."""
        if msg.enable and not self.support_box_removed:
            try:
                box_node = self.__robot.getFromDef('SUPPORT_BOX')
                if box_node:
                    box_node.remove()
                    self.support_box_removed = True
                    self._support_box_removed_step = self.loop_counter
                    self.__node.get_logger().info(
                        "[Experiment Mode] Support box removed! Robot is now free-standing."
                    )
                else:
                    self.__node.get_logger().warn(
                        "[Experiment Mode] DEF SUPPORT_BOX not found in world."
                    )
            except Exception as e:
                self.__node.get_logger().error(
                    f"[Experiment Mode] Failed to remove support box: {e}"
                )

    # Motor Command callback
    def cb_motor(self, msg):
        """
        當收到 C++ 發來的 MotorCmdStamped 時觸發
        """
        self.ros_control_active = True # 標記：開始使用 ROS 控制
        
        # 1. 解析訊息（包含 PD 增益、前饋扭矩、及 ABAD gamma）
        CMDS = {
            "A_Theta": msg.module_a.theta, 
            "A_Beta": msg.module_a.beta,
            "A_Gamma": msg.module_a.gamma,
            "A_kp_r": msg.module_a.kp_r,
            "A_kp_l": msg.module_a.kp_l,
            "A_kp_h": msg.module_a.kp_h,
            "A_kd_r": msg.module_a.kd_r,
            "A_kd_l": msg.module_a.kd_l,
            "A_kd_h": msg.module_a.kd_h,
            "A_torque_r": msg.module_a.torque_r,
            "A_torque_l": msg.module_a.torque_l,
            "A_torque_h": msg.module_a.torque_h,
            
            "B_Theta": msg.module_b.theta, 
            "B_Beta": msg.module_b.beta,
            "B_Gamma": msg.module_b.gamma,
            "B_kp_r": msg.module_b.kp_r,
            "B_kp_l": msg.module_b.kp_l,
            "B_kp_h": msg.module_b.kp_h,
            "B_kd_r": msg.module_b.kd_r,
            "B_kd_l": msg.module_b.kd_l,
            "B_kd_h": msg.module_b.kd_h,
            "B_torque_r": msg.module_b.torque_r,
            "B_torque_l": msg.module_b.torque_l,
            "B_torque_h": msg.module_b.torque_h,
            
            "C_Theta": msg.module_c.theta, 
            "C_Beta": msg.module_c.beta,
            "C_Gamma": msg.module_c.gamma,
            "C_kp_r": msg.module_c.kp_r,
            "C_kp_l": msg.module_c.kp_l,
            "C_kp_h": msg.module_c.kp_h,
            "C_kd_r": msg.module_c.kd_r,
            "C_kd_l": msg.module_c.kd_l,
            "C_kd_h": msg.module_c.kd_h,
            "C_torque_r": msg.module_c.torque_r,
            "C_torque_l": msg.module_c.torque_l,
            "C_torque_h": msg.module_c.torque_h,
            
            "D_Theta": msg.module_d.theta, 
            "D_Beta": msg.module_d.beta,
            "D_Gamma": msg.module_d.gamma,
            "D_kp_r": msg.module_d.kp_r,
            "D_kp_l": msg.module_d.kp_l,
            "D_kp_h": msg.module_d.kp_h,
            "D_kd_r": msg.module_d.kd_r,
            "D_kd_l": msg.module_d.kd_l,
            "D_kd_h": msg.module_d.kd_h,
            "D_torque_r": msg.module_d.torque_r,
            "D_torque_l": msg.module_d.torque_l,
            "D_torque_h": msg.module_d.torque_h,
        }
        # Update latest command
        self.latest_command = CMDS.copy()
        
    def execute_motor(self):
        
        # Use latest_command if available, otherwise do nothing (motors will keep their state).
        cmd = self.latest_command

        if cmd:
            # 處理四腿目標（使用固定 PID 參數）
            motor_debug_msg = "\n"
            for leg_id in ('A', 'B', 'C', 'D'):
                leg = self.legs[leg_id]
                # Position AND torque cross the same boundary together. They
                # used to disagree: the pose was mirrored for dir_beta = -1
                # legs while the feedforward torque was not.
                # NOTE: convert_torque / convert_gains are deliberately NOT
                # applied here. The argument for them is sound on paper --
                # dir_beta is honoured on position and state but not on the
                # gains or feedforward -- but measured, both the torque-only
                # and torque+gains variants make the gait markedly worse, and
                # they invert the left/right ratio (1.34-1.47 -> ~0.72, close
                # to its reciprocal) rather than centring it on 1.0. That is
                # the signature of a swap applied where none was missing.
                # See [[Torque Frame Mismatch — dir_beta vs Feedforward]].
                # The methods and their tests are kept as the record.
                theta, beta = leg.ros_to_module(cmd[f"{leg_id}_Theta"],
                                                cmd[f"{leg_id}_Beta"])
                motor_debug_msg += leg.set_target(
                    theta, beta,
                    cmd[f"{leg_id}_kp_r"], cmd[f"{leg_id}_kp_l"],
                    cmd[f"{leg_id}_kd_r"], cmd[f"{leg_id}_kd_l"],
                    cmd[f"{leg_id}_torque_r"] + self.trq_feedforward,
                    cmd[f"{leg_id}_torque_l"] + self.trq_feedforward,
                )
                leg.set_abad(
                    cmd[f"{leg_id}_Gamma"],
                    cmd[f"{leg_id}_kp_h"],
                    cmd[f"{leg_id}_kd_h"],
                    cmd[f"{leg_id}_torque_h"],
                )
                leg.update_g_joint(theta, beta)
                motor_debug_msg += "\n"
            
            # 顯示扭矩控制參數（使用固定 PID 值）
            self.__node.get_logger().debug(
                motor_debug_msg
            )

        else:
            # If no command has ever been received, set to default position
            for leg_id in ('A', 'B', 'C', 'D'):
                leg = self.legs[leg_id]
                theta, beta = leg.ros_to_module(self.default_theta,
                                                self.default_beta)
                leg.set_target(
                    theta, beta,
                    self.KP, self.KP,
                    self.KD, self.KD,
                    0.0, 0.0,
                )
                leg.set_abad(self.default_gamma, self.GAMMA_KP, self.GAMMA_KD, 0.0)
    
    def pub_tf(self):
        # B. 發布 TF (完美的里程計)
        if self.__self_node:
            # 取得絕對位置 (X, Y, Z)
            pos = self.__self_node.getPosition()
            # 取得旋轉 (Axis-Angle: [x, y, z, angle])
            rot_field = self.__self_node.getField("rotation")
            if rot_field:
                rot = rot_field.getSFRotation()
                # 將 Axis-Angle 轉換為 Quaternion (x, y, z, w)
                half_angle = rot[3] / 2
                sin_half = math.sin(half_angle)
                
                t = TransformStamped()
                # ros_time_msg = Time()
                t.header.stamp = self.ros_time_msg
                t.header.frame_id = "odom"       # 父座標 (世界)
                t.child_frame_id = "base_link"   # 子座標 (機器人本體)
                
                t.transform.translation.x = pos[0]
                t.transform.translation.y = pos[1]
                t.transform.translation.z = pos[2]
                
                # 計算四元數
                t.transform.rotation.x = rot[0] * sin_half
                t.transform.rotation.y = rot[1] * sin_half
                t.transform.rotation.z = rot[2] * sin_half
                t.transform.rotation.w = math.cos(half_angle)
                
                self.tf_broadcaster.sendTransform(t)
    
    def pub_imu(self):
        time_stamp = Time()
        time_stamp.sec = int(self.__robot.getTime())
        time_stamp.nanosec = int((self.__robot.getTime() - int(self.__robot.getTime())) * 1e9)
        imu_msg = self.imu_sensor.get_msg(time_stamp, self.loop_counter)
        self.imu_sensor.imu_pub.publish(imu_msg)
    
    def pub_clock(self):
        """pub sim clock to /clock topic"""
        now = self.__robot.getTime()
        
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        
        # handle nanosecond overflow
        if nsec >= 1000000000:
            sec += 1
            nsec = nsec % 1000000000
        elif nsec < 0:
            nsec = 0
        
        self.ros_time_msg.sec = sec
        self.ros_time_msg.nanosec = nsec
        
        clock_msg = Clock()
        clock_msg.clock = self.ros_time_msg
        self.clock_pub.publish(clock_msg)
    
    def pub_motor_state(self):
        motor_state_msg = MotorStateStamped()
        motor_state_msg.header.seq = self.loop_counter
        motor_state_msg.header.stamp = self.ros_time_msg
        # 取得所有馬達狀態
        motor_state_msg.module_a = self.legs['A'].get_states()
        motor_state_msg.module_b = self.legs['B'].get_states()
        motor_state_msg.module_c = self.legs['C'].get_states()
        motor_state_msg.module_d = self.legs['D'].get_states()
        self.motor_state_pub.publish(motor_state_msg)
    
    def pub_fsm(self):
        """Publish robot state with standby mode"""
        fsm_msg = RobotStateStamped()
        fsm_msg.header.seq = self.loop_counter
        fsm_msg.header.stamp = self.ros_time_msg
        fsm_msg.robot_mode = 3  # standby mode
        self.fsm_pub.publish(fsm_msg)

    def pub_base_odom(self):
        """Publish Supervisor ground-truth body pose and velocity."""
        if not self.__self_node:
            return
        if self.loop_counter % self._odom_update_interval != 0:
            return

        try:
            pos = self.__self_node.getPosition()
            # [vx, vy, vz, wx, wy, wz], world frame
            vel = self.__self_node.getVelocity()
            # Row-major, maps body -> world (same matrix pub_contact uses)
            R = self.__self_node.getOrientation()
        except Exception as e:
            if self.loop_counter % 5000 == 0:
                self.__node.get_logger().warn(f"[Odom] error: {e}")
            return

        msg = Odometry()
        msg.header.stamp = self.ros_time_msg
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = pos[2]

        # Reuse the axis-angle -> quaternion conversion pub_tf does, so pose
        # here and the TF frame can never disagree.
        rot_field = self.__self_node.getField("rotation")
        if rot_field:
            rot = rot_field.getSFRotation()
            half_angle = rot[3] / 2
            sin_half = math.sin(half_angle)
            msg.pose.pose.orientation.x = rot[0] * sin_half
            msg.pose.pose.orientation.y = rot[1] * sin_half
            msg.pose.pose.orientation.z = rot[2] * sin_half
            msg.pose.pose.orientation.w = math.cos(half_angle)

        # World -> body is R transposed; indexing matches pub_contact.
        vx, vy, vz = vel[0], vel[1], vel[2]
        wx, wy, wz = vel[3], vel[4], vel[5]
        msg.twist.twist.linear.x = R[0]*vx + R[3]*vy + R[6]*vz
        msg.twist.twist.linear.y = R[1]*vx + R[4]*vy + R[7]*vz
        msg.twist.twist.linear.z = R[2]*vx + R[5]*vy + R[8]*vz
        msg.twist.twist.angular.x = R[0]*wx + R[3]*wy + R[6]*wz
        msg.twist.twist.angular.y = R[1]*wx + R[4]*wy + R[7]*wz
        msg.twist.twist.angular.z = R[2]*wx + R[5]*wy + R[8]*wz

        self.base_odom_pub.publish(msg)

    def log_foot_frame(self):
        """Diagnostic: where each foot sits in the BODY frame, per leg.

        Settles the beta sign convention empirically. Every check inside either
        codebase is done in the leg/module frame, and the modules are mounted
        with a 120 deg rotation (about (1,1,1) for A,B and (1,-1,-1) for C,D),
        so "beta > 0 puts the foot at +x" in the leg model says nothing about
        which way the body actually travels. This reports foot-minus-hip along
        body x: positive = the foot is AHEAD of its own hip.

        Off unless CORGI_FOOT_DEBUG=1; costs four Supervisor reads when on.
        """
        if not self._foot_debug or not self.__self_node:
            return
        if self.loop_counter % 500 != 0:
            return

        try:
            pos = self.__self_node.getPosition()
            R = self.__self_node.getOrientation()
        except Exception:
            return

        # Centre of mass in the BODY frame. The fore/aft COM offset is on
        # record (1.7 mm behind the wheelbase midpoint, from weighing the real
        # robot) but the LATERAL offset has never been checked, and a lateral
        # COM bias is a candidate for the constant per-stride yaw. Supervisor
        # getCenterOfMass returns world coordinates; rotate into the body.
        try:
            com = self.__self_node.getCenterOfMass()
            cx, cy, cz = com[0]-pos[0], com[1]-pos[1], com[2]-pos[2]
            com_x = R[0]*cx + R[3]*cy + R[6]*cz
            com_y = R[1]*cx + R[4]*cy + R[7]*cz
            com_z = R[2]*cx + R[5]*cy + R[8]*cz
            self.__node.get_logger().info(
                f"[COM] body frame x={com_x*1000:+.2f} mm  y={com_y*1000:+.2f} mm"
                f"  z={com_z*1000:+.2f} mm   (y is LATERAL: +y is left)")
        except Exception as e:
            self.__node.get_logger().warn(f"[COM] unavailable: {e}")

        parts = []
        for leg_id in ('A', 'B', 'C', 'D'):
            pair = self._foot_nodes.get(leg_id)
            if not pair:
                continue
            pr, pl = pair[0].getPosition(), pair[1].getPosition()
            fp = [0.5 * (a + b) for a, b in zip(pr, pl)]
            dx, dy, dz = fp[0]-pos[0], fp[1]-pos[1], fp[2]-pos[2]
            # world -> body, same indexing as pub_contact
            bx = R[0]*dx + R[3]*dy + R[6]*dz
            by = R[1]*dx + R[4]*dy + R[7]*dz
            bz = R[2]*dx + R[5]*dy + R[8]*dz
            hip_x, hip_y = self._MODULE_XY[leg_id]

            # Measured joint angles alongside, so the actual foot position can
            # be checked against what the leg model predicts FROM THOSE ANGLES.
            # Disagreement => the Webots geometry and LegModel disagree for
            # this leg. Agreement => the angles themselves are off, i.e. a
            # tracking or load effect. Leg A shows a -40 mm fore-aft offset and
            # ~70% of the others' beta sensitivity; this separates the two.
            leg = self.legs[leg_id]
            th, be = leg.module_to_ros(
                *leg.tb.FK(leg.sensors["L_Motor"].getValue(),
                           leg.sensors["R_Motor"].getValue()))
            parts.append(
                f"{leg_id}: th={math.degrees(th):7.3f} be={math.degrees(be):+7.3f} "
                f"dx={bx-hip_x:+.4f} dy={by-hip_y:+.4f} dz={bz:+.4f}")

        if parts:
            self.__node.get_logger().info("[FootFrame] " + " | ".join(parts))

    def pub_contact(self):
        msg = SimLegContactStamped()
        msg.header.seq = self.loop_counter
        msg.header.stamp = self.ros_time_msg

        contact_set = set()
        n_cp_total  = 0
        n_cp_ground = 0

        # Skip for 1 step immediately after support box removal (avoid Webots segfault on stale physics)
        if self.loop_counter <= self._support_box_removed_step + 1:
            self.contact_pub.publish(msg)
            return

        # Only refresh contact points every N steps
        if self.loop_counter % self._contact_update_interval == 0:
            contact_set = set()
            try:
                pos = self.__self_node.getPosition()
                R   = self.__self_node.getOrientation()
                cps = self.__self_node.getContactPoints(includeDescendants=True)
                n_cp_total = len(cps)

                for cp in cps:
                    pw = cp.getPoint()
                    if pw[2] > 0.02:
                        continue
                    n_cp_ground += 1
                    # Transform to robot body frame
                    dx, dy, dz = pw[0]-pos[0], pw[1]-pos[1], pw[2]-pos[2]
                    bx = R[0]*dx + R[3]*dy + R[6]*dz
                    by = R[1]*dx + R[4]*dy + R[7]*dz
                    # Assign to nearest module (avoids cross-quadrant bleed from arc geometry)
                    best, best_d = 'A', float('inf')
                    for leg_id, (mx, my) in self._MODULE_XY.items():
                        d = (bx-mx)**2 + (by-my)**2
                        if d < best_d:
                            best_d, best = d, leg_id
                    contact_set.add(best)

                self._contact_cache = contact_set
                if self.loop_counter % 1000 == 0:
                    self.__node.get_logger().info(
                        f"[Contact] legs={contact_set} n_cp={n_cp_total} ground={n_cp_ground}"
                    )
            except Exception as e:
                if self.loop_counter % 5000 == 0:
                    self.__node.get_logger().warn(f"[Contact] error: {e}")
        else:
            contact_set = self._contact_cache

        module_map = {
            'A': msg.module_a, 'B': msg.module_b,
            'C': msg.module_c, 'D': msg.module_d,
        }
        for leg_id, module in module_map.items():
            module.contact = leg_id in contact_set
            module.rim_ul = module.contact
            module.rim_ll = module.contact
            module.rim_lr = module.contact
            module.rim_ur = module.contact

        self.contact_pub.publish(msg)

    # Webots main loop, Webots will call this function
    def step(self):
        # === 1. pub clock ===
        self.pub_clock()

        # === 2. process ros2 communication ===
        rclpy.spin_once(self.__node, timeout_sec=0)

        # === 3. control logic  ===
        self.execute_motor()

        # === 4. pub datas ===
        # TF
        self.pub_tf()
        # Motor State
        self.pub_motor_state()
        # IMU
        self.pub_imu()
        # Contact State
        self.pub_contact()
        # Body odometry (ground truth)
        self.pub_base_odom()
        # Foot-frame sign diagnostic (no-op unless CORGI_FOOT_DEBUG=1)
        self.log_foot_frame()
        # FSM
        self.pub_fsm()
        
        self.loop_counter += 1
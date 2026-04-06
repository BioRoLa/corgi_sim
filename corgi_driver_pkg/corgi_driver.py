import rclpy
import math
import os

# --- Webots 控制器模組 (用於控制模擬狀態) ---
from controller import Supervisor

# --- [新增] Supervisor 與 TF 相關模組 ---
from rosgraph_msgs.msg import Clock
from corgi_msgs.msg import TriggerStamped
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from tf2_ros import TransformBroadcaster
from corgi_msgs.msg import MotorCmdStamped
from corgi_msgs.msg import MotorStateStamped, MotorState
from corgi_msgs.msg import ImuStamped
from corgi_msgs.msg import RobotStateStamped

from . import Controller_TB
from .LegModel import LegModel

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

class LegManager:
    def __init__(self, robot, prefix, controller_tb, basic_time_step=1, Max_Torque=35,
                 abad_prefix=None, axis_dir=1):
        self.prefix = prefix
        self.motors = {}
        self.sensors = {}
        self.tb = controller_tb
        self.axis_dir = axis_dir
        motor_names = ["L_Motor", "R_Motor"]
        self.prev_pos_l = None
        self.prev_pos_r = None
        self.prev_vel_l = 0.0
        self.prev_vel_r = 0.0
        # 當前速度（用於狀態發布，避免重複計算）
        self.current_vel_l = 0.0
        self.current_vel_r = 0.0
        # 扭矩命令存儲（用於發布命令值）
        self.cmd_trq_l = 0.0
        self.cmd_trq_r = 0.0
        self.cmd_trq_abad = 0.0
        self.basic_time_step = basic_time_step
        self.Max_Torque = Max_Torque
        
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
                motor.setAvailableTorque(self.Max_Torque)
        
        # --- ABAD 馬達 (位置控制) ---
        self.motor_abad = None
        self.sensor_abad = None
        self.prev_pos_abad = None
        self.prev_vel_abad = 0.0
        self.current_vel_abad = 0.0
        if abad_prefix:
            abad_name = f"{abad_prefix}_ABAD"
            abad_sensor_name = f"{abad_prefix}_ABAD_sensor"
            self.motor_abad = robot.getDevice(abad_name)
            if self.motor_abad:
                # 設定為扭矩控制模式（與 L/R Motor 一致）
                self.motor_abad.setPosition(float('inf'))
                self.motor_abad.setVelocity(0.0)
                self.motor_abad.enableTorqueFeedback(self.basic_time_step)
                self.motor_abad.setAvailableTorque(self.Max_Torque)
                self.sensor_abad = robot.getDevice(abad_sensor_name)
                if self.sensor_abad:
                    self.sensor_abad.enable(self.basic_time_step)
                else:
                    print(f"Warning: 找不到 ABAD Sensor {abad_sensor_name}")
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
        
        # 計算位置誤差（避免在扭矩計算中重複計算）
        err_l = cmd_L - pos_l
        err_r = cmd_R - pos_r
        
        # trq = kp * (phi_desired - phi_actual) + kd * (-phi_dot_actual) + torque_ff
        trq_r = kp_r * err_r + kd_r * (-vel_r) + torque_r * 1 if err_r else -1
        trq_l = kp_l * err_l + kd_l * (-vel_l) + torque_l * 1 if err_l else -1
        
        # 保存扭矩命令
        self.cmd_trq_r = trq_r
        self.cmd_trq_l = trq_l
        
        # 設定扭矩到 Webots 馬達
        if "R_Motor" in self.motors:
            if trq_r > self.Max_Torque:
                trq_r = self.Max_Torque
            elif trq_r < -self.Max_Torque:
                trq_r = -self.Max_Torque
            self.motors["R_Motor"].setTorque(trq_r)
        if "L_Motor" in self.motors:
            if trq_l > self.Max_Torque:
                trq_l = self.Max_Torque
            elif trq_l < -self.Max_Torque:
                trq_l = -self.Max_Torque
            self.motors["L_Motor"].setTorque(trq_l)
        
        # print debug info
        return "".join([f"[{self.prefix}] Target θ: {theta:.3f} rad, β: {beta:.3f} rad | ",
                                      f"Cmd L: {cmd_L:.3f} rad, R: {cmd_R:.3f} rad | ",
                                      f"Pos L: {pos_l:.3f} rad, R: {pos_r:.3f} rad | ",
                                      f"Err L: {err_l:.3f} rad, R: {err_r:.3f} rad | ",
                                      f"Vel L: {vel_l:.3f} rad/s, R: {vel_r:.3f} rad/s | ",
                                      f"Trq L: {trq_l:.3f} Nm, R: {trq_r:.3f} Nm"])
    
    def set_abad(self, gamma, kp=0.0, kd=0.0, torque_ff=0.0):
        """設定 ABAD 馬達扭矩 (扭矩控制)
        
        Args:
            gamma: ABAD 目標角度 (rad)
            kp: 位置比例增益
            kd: 速度阻尼增益
            torque_ff: 前饋扭矩 (N·m)
        """
        if self.motor_abad:
            # 與現有符號方向一致
            gamma_target = gamma * self.axis_dir

            pos_abad = self.sensor_abad.getValue()
            if self.prev_pos_abad is None:
                self.prev_pos_abad = pos_abad

            dt = self.basic_time_step / 1000.0
            vel_abad = (pos_abad - self.prev_pos_abad) / dt
            self.prev_pos_abad = pos_abad
            self.prev_vel_abad = vel_abad
            self.current_vel_abad = vel_abad

            # 處理角度連續性（避免 ±π 跳變）
            gamma_target = self._find_closest_phi(gamma_target, pos_abad)

            err_abad = gamma_target - pos_abad
            trq_abad = kp * err_abad + kd * (-vel_abad) + torque_ff

            if trq_abad > self.Max_Torque:
                trq_abad = self.Max_Torque
            elif trq_abad < -self.Max_Torque:
                trq_abad = -self.Max_Torque

            self.cmd_trq_abad = trq_abad
            self.motor_abad.setTorque(trq_abad)
            return "".join([
                f"ABAD Target: {gamma_target:.3f} rad | ",
                f"Pos: {pos_abad:.3f} rad | ",
                f"Err: {err_abad:.3f} rad | ",
                f"Vel: {vel_abad:.3f} rad/s | ",
                f"Trq: {trq_abad:.3f} Nm"
            ])

        return "ABAD motor not found"
    
    def update_g_joint(self, theta, beta):
        """根據 theta/beta 更新 G_Joint 被動追蹤
        
        Args:
            theta: 腿部伸展角 (rad)
            beta: 腿部旋轉角 (rad)
        """
        if self.motor_g_joint and self.leg_model:
            self.leg_model.theta = theta
            self.leg_model.beta = beta
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
        
        msg = MotorState()
        theta, beta = self.tb.FK(pos_l, pos_r)
        msg.theta, msg.beta = float(theta), float(beta)
        
        # 直接使用set_target中計算的速度（已經過濾波）
        msg.velocity_l = float(self.current_vel_l)
        msg.velocity_r = float(self.current_vel_r)

        # ABAD 狀態        
        msg.gamma = float(self.sensor_abad.getValue())
        msg.velocity_h = float(self.current_vel_abad)
        
        # 發布扭矩命令值（命令扭矩，而非回饋）
        msg.torque_r = float(self.cmd_trq_r)
        msg.torque_l = float(self.cmd_trq_l)
        msg.torque_h = float(self.cmd_trq_abad)
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
        # TUNED Params
        self.KP = 90.0
        self.KI = 0.0
        self.KD = 1.75
        # self.Max_Torque = self.KP if self.KP > 35.0 else 35.0
        self.Max_Torque = 35.0
        self.trq_feedforward = 0  # N·m 前饋扭矩
        
        # 3. initialize Legs
        self.tb_lib = Controller_TB.Controller_TB(theta_0=math.radians(17))
        # ABAD axis_dir 設定：與 CSV_Controller_12D 中一致
        # A(FL): -1, B(FR): 1, C(RR): -1, D(RL): 1
        self.legs = {
            'A': LegManager(self.__robot, "A_Module_", Controller_TB.Controller_TB(theta_0=math.radians(17)),
                             basic_time_step=self.__timestep, Max_Torque=self.Max_Torque,
                             abad_prefix='A', axis_dir=-1),
            'B': LegManager(self.__robot, "B_Module_", Controller_TB.Controller_TB(theta_0=math.radians(17)),
                             basic_time_step=self.__timestep, Max_Torque=self.Max_Torque,
                             abad_prefix='B', axis_dir=1),
            'C': LegManager(self.__robot, "C_Module_", Controller_TB.Controller_TB(theta_0=math.radians(17)),
                             basic_time_step=self.__timestep, Max_Torque=self.Max_Torque,
                             abad_prefix='C', axis_dir=-1),
            'D': LegManager(self.__robot, "D_Module_", Controller_TB.Controller_TB(theta_0=math.radians(17)),
                             basic_time_step=self.__timestep, Max_Torque=self.Max_Torque,
                             abad_prefix='D', axis_dir=1)
        }

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
        
        # Initialize loop counter
        self.loop_counter = 0
        
        # --- Experiment Mode ---
        self.experiment_mode = os.environ.get('CORGI_EXPERIMENT_MODE', '0') == '1'
        self.support_box_removed = False
        
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
            motor_debug_msg += self.legs['A'].set_target(
                cmd["A_Theta"], -cmd["A_Beta"],
                self.KP, self.KP,
                self.KD, self.KD,
                cmd["A_torque_r"] + self.trq_feedforward, cmd["A_torque_l"] + self.trq_feedforward
            )
            motor_debug_msg += " | " + self.legs['A'].set_abad(
                cmd["A_Gamma"], cmd["A_kp_h"], cmd["A_kd_h"], cmd["A_torque_h"]
            )
            self.legs['A'].update_g_joint(cmd["A_Theta"], -cmd["A_Beta"])
            motor_debug_msg += "\n"
            motor_debug_msg += self.legs['B'].set_target(
                cmd["B_Theta"], cmd["B_Beta"],
                self.KP, self.KP,
                self.KD, self.KD,
                cmd["B_torque_r"] + self.trq_feedforward, cmd["B_torque_l"] + self.trq_feedforward
            )
            motor_debug_msg += " | " + self.legs['B'].set_abad(
                cmd["B_Gamma"], cmd["B_kp_h"], cmd["B_kd_h"], cmd["B_torque_h"]
            )
            self.legs['B'].update_g_joint(cmd["B_Theta"], -cmd["B_Beta"])
            motor_debug_msg += "\n"
            motor_debug_msg += self.legs['C'].set_target(
                cmd["C_Theta"], cmd["C_Beta"],
                self.KP, self.KP,
                self.KD, self.KD,
                cmd["C_torque_r"] + self.trq_feedforward, cmd["C_torque_l"] + self.trq_feedforward
            )
            motor_debug_msg += " | " + self.legs['C'].set_abad(
                cmd["C_Gamma"], cmd["C_kp_h"], cmd["C_kd_h"], cmd["C_torque_h"]
            )
            self.legs['C'].update_g_joint(cmd["C_Theta"], -cmd["C_Beta"])
            motor_debug_msg += "\n"
            motor_debug_msg += self.legs['D'].set_target(
                cmd["D_Theta"], -cmd["D_Beta"],
                self.KP, self.KP,
                self.KD, self.KD,
                cmd["D_torque_r"] + self.trq_feedforward, cmd["D_torque_l"] + self.trq_feedforward
            )
            motor_debug_msg += " | " + self.legs['D'].set_abad(
                cmd["D_Gamma"], cmd["D_kp_h"], cmd["D_kd_h"], cmd["D_torque_h"]
            )
            self.legs['D'].update_g_joint(cmd["D_Theta"], -cmd["D_Beta"])
            
            # 顯示扭矩控制參數（使用固定 PID 值）
            self.__node.get_logger().debug(
                motor_debug_msg
            )

        else:
            # If no command has ever been received, set to default position
            self.legs['A'].set_target(
                self.default_theta, self.default_beta,
                self.KP, self.KP,
                self.KD, self.KD,
                0.0, 0.0
            )
            self.legs['A'].set_abad(0.0, self.KP, self.KD, 0.0)
            self.legs['B'].set_target(
                self.default_theta, self.default_beta,
                self.KP, self.KP,
                self.KD, self.KD,
                0.0, 0.0
            )
            self.legs['B'].set_abad(0.0, self.KP, self.KD, 0.0)
            self.legs['C'].set_target(
                self.default_theta, self.default_beta,
                self.KP, self.KP,
                self.KD, self.KD,
                0.0, 0.0
            )
            self.legs['C'].set_abad(0.0, self.KP, self.KD, 0.0)
            self.legs['D'].set_target(
                self.default_theta, self.default_beta,
                self.KP, self.KP,
                self.KD, self.KD,
                0.0, 0.0
            )
            self.legs['D'].set_abad(0.0, self.KP, self.KD, 0.0)
    
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
        # FSM
        self.pub_fsm()
        
        self.loop_counter += 1
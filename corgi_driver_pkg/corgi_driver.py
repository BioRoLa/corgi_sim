import rclpy
import math
import csv
import os
from ament_index_python.packages import get_package_share_directory

# --- Webots 控制器模組 (用於控制模擬狀態) ---
from controller import Supervisor

# --- [新增] Supervisor 與 TF 相關模組 ---
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from tf2_ros import TransformBroadcaster
from corgi_msgs.msg import MotorCmdStamped
from corgi_msgs.msg import TriggerStamped
from corgi_msgs.msg import MotorStateStamped, MotorState
from corgi_msgs.msg import ImuStamped

from . import Controller_TB
# -------------------------------------
# Read_CSV = not False
Read_CSV = False

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
    def __init__(self, robot, prefix, controller_tb, basic_time_step=1):
        self.prefix = prefix
        self.motors = {}
        self.sensors = {}
        self.tb = controller_tb
        motor_names = ["L_Motor", "R_Motor"]
        self.prev_pos_l = None
        self.prev_pos_r = None
        self.prev_vel_l = 0.0
        self.prev_vel_r = 0.0
        self.basic_time_step = basic_time_step
        
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
                try:
                    motor.setAvailableTorque(0.0)
                except:
                    pass

    def set_target(self, theta, beta, kp_r=0.0, kp_l=0.0, kd_r=0.0, kd_l=0.0, torque_r=0.0, torque_l=0.0):
        """
        設定腿部目標 - 純扭矩控制（參考 ROS1 corgi_sim_trq.cpp）
        
        Args:
            theta: 腿部伸展角 (rad)
            beta: 腿部旋轉角 (rad)
            kp_r, kp_l: 位置比例增益
            kd_r, kd_l: 速度阻尼增益
            torque_r, torque_l: 前饋扭矩 (N·m)
        
        控制律 (與 ROS1 相同):
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
        
        # 初始化前一時刻的位置
        if self.prev_pos_r is None:
            self.prev_pos_r = pos_r
        if self.prev_pos_l is None:
            self.prev_pos_l = pos_l
        
        # 計算速度 (rad/s) - 使用數值微分
        dt = self.basic_time_step / 1000.0  # 轉換為秒
        vel_r = (pos_r - self.prev_pos_r) / dt
        vel_l = (pos_l - self.prev_pos_l) / dt
        
        # 更新歷史位置
        self.prev_pos_r = pos_r
        self.prev_pos_l = pos_l
        self.prev_vel_r = vel_r
        self.prev_vel_l = vel_l
        
        # 處理角度連續性（避免 ±π 跳變）
        cmd_R = self._find_closest_phi(cmd_R, pos_r)
        cmd_L = self._find_closest_phi(cmd_L, pos_l)
        
        # === ROS1 corgi_sim_trq.cpp Line 115-116 控制律 ===
        # trq = kp * (phi_desired - phi_actual) + kd * (-phi_dot_actual) + torque_ff
        trq_r = kp_r * (cmd_R - pos_r) + kd_r * (-vel_r) + torque_r
        trq_l = kp_l * (cmd_L - pos_l) + kd_l * (-vel_l) + torque_l
        
        # 設定扭矩到 Webots 馬達
        if "R_Motor" in self.motors:
            self.motors["R_Motor"].setTorque(trq_r)
        if "L_Motor" in self.motors:
            self.motors["L_Motor"].setTorque(trq_l)
    
    def _find_closest_phi(self, phi_target, phi_current):
        """
        找到最接近的等價角度（處理 2π 週期性）
        """
        diff = (phi_target - phi_current + math.pi) % (2 * math.pi) - math.pi
        return phi_current + diff
    
    def get_positions(self):
        positions = {}
        for name, sensor in self.sensors.items():
            positions[self.prefix + name] = sensor.getValue()
        return positions
    
    def get_velocities(self):
        velocities = {}
        for name, motor in self.motors.items():
            velocities[self.prefix + name] = motor.getVelocity()
        return velocities
    
    def get_torques(self):
        torques = {}
        for name, motor in self.motors.items():
            torques[self.prefix + name] = motor.getTorqueFeedback()
        return torques
    
    
    def get_states(self):
        pos_l = self.sensors["L_Motor"].getValue()
        pos_r = self.sensors["R_Motor"].getValue()
        if self.prev_pos_l is None:
            self.prev_pos_l = pos_l
        if self.prev_pos_r is None:
            self.prev_pos_r = pos_r
        vel_l = (pos_l - self.prev_pos_l) / self.basic_time_step * 1000.0
        vel_r = (pos_r - self.prev_pos_r) / self.basic_time_step * 1000.0
        self.prev_pos_l = pos_l
        self.prev_pos_r = pos_r
        msg = MotorState()
        theta, beta = self.tb.FK(pos_l,pos_r)
        msg.theta , msg.beta = theta, beta
        msg.velocity_r = vel_r
        msg.velocity_l = vel_l
        msg.torque_r = self.motors["R_Motor"].getTorqueFeedback()
        msg.torque_l = self.motors["L_Motor"].getTorqueFeedback()
        return msg
    
class CorgiDriver:
    def init(self, webots_node, properties):
        # 1. 取得 Webots 機器人
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        # [新增] 暫停旗標：確保只會自動暫停一次
        self.has_paused = False
        # 2. 初始化 ROS 2 並建立我們自己的 Node
        # 先檢查是否已經 init 過，避免重複報錯
        if not rclpy.ok():
            rclpy.init(args=None)
            
        # 建立名為 corgi_driver 的節點
        self.__node = rclpy.create_node('corgi_driver_internal')
        
        # 取得自己在 Webots 中的節點 (需要 World 裡 supervisor=TRUE)
        self.__self_node = self.__robot.getSelf()
        
        # 建立 /clock 發布器
        self.clock_pub = self.__node.create_publisher(Clock, 'clock', 1000)
        
        # 建立 TF 廣播器 (讓 Rviz 知道機器人在哪)
        self.tf_broadcaster = TransformBroadcaster(self.__node) 
        
        # 3. 初始化運動學
        self.tb_lib = Controller_TB.Controller_TB(theta_0=math.radians(17))
        self.legs = {
            'A': LegManager(self.__robot, "A_Module_", Controller_TB.Controller_TB(theta_0=math.radians(17)),
                             basic_time_step=self.__timestep),
            'B': LegManager(self.__robot, "B_Module_", Controller_TB.Controller_TB(theta_0=math.radians(17)),
                             basic_time_step=self.__timestep),
            'C': LegManager(self.__robot, "C_Module_", Controller_TB.Controller_TB(theta_0=math.radians(17)),
                             basic_time_step=self.__timestep),
            'D': LegManager(self.__robot, "D_Module_", Controller_TB.Controller_TB(theta_0=math.radians(17)),
                             basic_time_step=self.__timestep)
        }
        # 4. 初始化 IMU
        self.imu_sensor = imu(self.__robot, self.__node, basic_time_step=self.__timestep)
        
        # 5. Motor Command Subscriber
        self.motor_sub = self.__node.create_subscription(
            MotorCmdStamped,
            'motor/command',
            self.motor_callback,
            1000
        )
        # ROS CMD Buffer from motor callback
        self.ROS_CMD_Buffer = []
        
        # ROS Control Mode Flag
        self.trigger_pub = self.__node.create_publisher(
            TriggerStamped,
            "trigger",
            1000
        )
        self.trigger_msg = TriggerStamped()
        # motor state publisher
        self.motor_state_pub = self.__node.create_publisher(
            MotorStateStamped,
            'motor/state',
            1000
        )
        
        
        if Read_CSV:
            # 4. 讀取 CSV
            self.trajectory_data = []
            self.current_index = 0
            try:
                pkg_path = get_package_share_directory('corgi_ros_control')
                csv_path = os.path.join(pkg_path, 'resource', 'COT_Exp_Index_140_OLD.csv')
                
                # 使用我們自己的 node logger，現在這行不會報錯了！
                self.__node.get_logger().info(f"Loading CSV: {csv_path}")
                
                with open(csv_path, 'r') as file:
                    reader = csv.reader(file)
                    for row in reader:
                        try:
                            self.trajectory_data.append([float(val) for val in row])
                        except ValueError: continue
                self.__node.get_logger().info(f"✅ CSV Loaded: {len(self.trajectory_data)} rows")
            except Exception as e:
                self.__node.get_logger().error(f"❌ CSV Error: {str(e)}")
        else:       #ROS Control Mode
            self.__node.get_logger().info("⚠️ CSV Reading Disabled")
            # public trigger message enable = True
            self.trigger_msg.enable = True
            self.trigger_pub.publish(self.trigger_msg)
            self.current_index = 0
        self.__node.get_logger().info("🚀 Driver Initialized! Waiting for Play button...")
        
    # 4. [新增] Motor Command 回調
    def motor_callback(self, msg):
        """
        當收到 C++ 發來的 MotorCmdStamped 時觸發
        """
        self.ros_control_active = True # 標記：開始使用 ROS 控制
        
        # 1. 解析訊息（包含 PD 增益和前饋扭矩）
        CMDS = {
            "A_Theta": msg.module_a.theta, 
            "A_Beta": msg.module_a.beta,
            "A_kp_r": msg.module_a.kp_r,
            "A_kp_l": msg.module_a.kp_l,
            "A_kd_r": msg.module_a.kd_r,
            "A_kd_l": msg.module_a.kd_l,
            "A_torque_r": msg.module_a.torque_r,
            "A_torque_l": msg.module_a.torque_l,
            
            "B_Theta": msg.module_b.theta, 
            "B_Beta": msg.module_b.beta,
            "B_kp_r": msg.module_b.kp_r,
            "B_kp_l": msg.module_b.kp_l,
            "B_kd_r": msg.module_b.kd_r,
            "B_kd_l": msg.module_b.kd_l,
            "B_torque_r": msg.module_b.torque_r,
            "B_torque_l": msg.module_b.torque_l,
            
            "C_Theta": msg.module_c.theta, 
            "C_Beta": msg.module_c.beta,
            "C_kp_r": msg.module_c.kp_r,
            "C_kp_l": msg.module_c.kp_l,
            "C_kd_r": msg.module_c.kd_r,
            "C_kd_l": msg.module_c.kd_l,
            "C_torque_r": msg.module_c.torque_r,
            "C_torque_l": msg.module_c.torque_l,
            
            "D_Theta": msg.module_d.theta, 
            "D_Beta": msg.module_d.beta,
            "D_kp_r": msg.module_d.kp_r,
            "D_kp_l": msg.module_d.kp_l,
            "D_kd_r": msg.module_d.kd_r,
            "D_kd_l": msg.module_d.kd_l,
            "D_torque_r": msg.module_d.torque_r,
            "D_torque_l": msg.module_d.torque_l,
        }
        # Add to ROS CMD Buffer
        self.ROS_CMD_Buffer += [CMDS.copy()]
        
    def execute(self):
        if self.current_index < len(self.ROS_CMD_Buffer):
            cmd = self.ROS_CMD_Buffer[self.current_index]
            # 處理四腿目標（支援扭矩控制參數）
            self.legs['A'].set_target(
                cmd["A_Theta"], -cmd["A_Beta"],
                cmd["A_kp_r"], cmd["A_kp_l"],
                cmd["A_kd_r"], cmd["A_kd_l"],
                cmd["A_torque_r"], cmd["A_torque_l"]
            )
            self.legs['B'].set_target(
                cmd["B_Theta"], -cmd["B_Beta"],
                cmd["B_kp_r"], cmd["B_kp_l"],
                cmd["B_kd_r"], cmd["B_kd_l"],
                cmd["B_torque_r"], cmd["B_torque_l"]
            )
            self.legs['C'].set_target(
                cmd["C_Theta"], -cmd["C_Beta"],
                cmd["C_kp_r"], cmd["C_kp_l"],
                cmd["C_kd_r"], cmd["C_kd_l"],
                cmd["C_torque_r"], cmd["C_torque_l"]
            )
            self.legs['D'].set_target(
                cmd["D_Theta"], -cmd["D_Beta"],
                cmd["D_kp_r"], cmd["D_kp_l"],
                cmd["D_kd_r"], cmd["D_kd_l"],
                cmd["D_torque_r"], cmd["D_torque_l"]
            )
            
            # 顯示扭矩控制參數
            self.__node.get_logger().info(
                f'[TORQUE] A: θ={cmd["A_Theta"]:.3f}, β={cmd["A_Beta"]:.3f}, '
                f'kp={cmd["A_kp_r"]:.1f}, kd={cmd["A_kd_r"]:.2f}, '
                f'τ={cmd["A_torque_r"]:.3f}'
            )
            
            self.current_index += 1
    
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
                ros_time_msg = Time()
                t.header.stamp = ros_time_msg
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
        imu_msg = self.imu_sensor.get_msg(time_stamp, self.current_index)
        self.imu_sensor.imu_pub.publish(imu_msg)
    
    def pub_clock(self):
        now = self.__robot.getTime()
        ros_time_msg = Time()
        ros_time_msg.sec = int(now) 
        ros_time_msg.nanosec = int((now - int(now)) * 1e9)
        self.clock_pub.publish(Clock(clock=ros_time_msg))
    
    def motor_state_publish(self):
        motor_state_msg = MotorStateStamped()
        motor_state_msg.header.seq = self.current_index
        # 取得所有馬達狀態
        motor_state_msg.module_a = self.legs['A'].get_states()
        motor_state_msg.module_b = self.legs['B'].get_states()
        motor_state_msg.module_c = self.legs['C'].get_states()
        motor_state_msg.module_d = self.legs['D'].get_states()
        self.motor_state_pub.publish(motor_state_msg)
    
    # 5. [回歸標準] 使用 step 回調
    # Webots 外部驅動程式會不斷呼叫這個函式
    def step(self):
        # 讓 ROS 2 處理通訊 (這會讓 Logger 和 Topic 有作用)
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        # A. 發布模擬時間 /clock
        self.pub_clock()
        # B. 發布 TF
        self.pub_tf()
        # C. 發布 Motor State
        self.motor_state_publish()
        # D. 發布 IMU 資料
        self.pub_imu()
        # ---------------------------------
        now = self.__robot.getTime()
        if Read_CSV:
            # 如果時間超過 5 秒，且「之前還沒暫停過」
            if now >= 5.0 and not self.has_paused:
                self.__node.get_logger().warn("⏸️ Time is up (5s)! Pausing Simulation...")
                
                # 設定模擬模式為 PAUSE (暫停)
                self.__robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
                
                # [重要] 標記為已暫停，這樣當您手動按 Play 繼續時，才不會又卡住
                self.has_paused = True
            # ---------------------------------
            # 存活確認 Log (每 1 秒印一次)
            # 如果這一行有印出來，代表「程式在跑」
            if int(now * 1000) % 1000 == 0:
                self.__node.get_logger().info(f"🟢 Running... Time: {now:.2f}s | Idx: {self.current_index}")
            # 播放 CSV
            if self.current_index < len(self.trajectory_data):
                row = self.trajectory_data[self.current_index]
                self.legs['A'].set_target(row[0], -row[1])
                self.legs['B'].set_target(row[2], -row[3])
                self.legs['C'].set_target(row[4], -row[5])
                self.legs['D'].set_target(row[6], -row[7])
                self.__node.get_logger().info("".join([ f"\nReceived CMD: A( {row[0]:.5f}, {row[1]:.2f})\n",
                                                        f"Received CMD: B( {row[2]:.5f}, {row[3]:.2f})\n",
                                                        f"Received CMD: C( {row[4]:.5f}, {row[5]:.2f})\n",
                                                        f"Received CMD: D( {row[6]:.5f}, {row[7]:.2f})\n"]))
                self.current_index += 1
        else:
            self.execute()
            # if self.ROS_CMD_Buffer:
            #     self.execute()
            # public trigger message enable = True
            self.trigger_pub.publish(self.trigger_msg)
    # def step(self):
    #     rclpy.spin_once(self.__node, timeout_sec=0)
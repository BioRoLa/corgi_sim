import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from builtin_interfaces.msg import Time
from controller import Robot

class ForcePlateDriver:
    def init(self, webots_node, properties):
        """
        Initialize the ROS 2 node and Force Plate sensors.
        Referencing the structure from corgi_driver.py.
        """
        # 1. Get Webots robot instance
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # 2. Initialize ROS 2 Node
        # Check if rclpy is already initialized
        if not rclpy.ok():
            rclpy.init(args=None)
        
        self.__node = rclpy.create_node('force_plate_driver')
        
        # 3. Initialize Sensors and Publishers
        # We have 4 force plates, creating a publisher for each
        self.force_plates = []
        self.publishers = []
        
        for i in range(1, 5):
            # Webots Device
            plate_name = f'force_plate_{i}'
            plate = self.__robot.getDevice(plate_name)
            if plate:
                plate.enable(self.__timestep)
                self.force_plates.append(plate)
                
                # ROS 2 Publisher
                pub = self.__node.create_publisher(
                    WrenchStamped, 
                    f'sensor/{plate_name}', 
                    1
                )
                self.publishers.append(pub)
            else:
                self.__node.get_logger().error(f"Sensor {plate_name} not found!")

        self.__node.get_logger().info("Force Plate Driver Initialized with ROS 2 Topics!")

    def step(self):
        """
        Main step function called by Webots.
        Reads sensor data and publishes to ROS 2.
        """
        # 1. Process ROS 2 events (if any)
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        # 2. Get current simulation time for timestamping
        now = self.__robot.getTime()
        stamp = Time()
        stamp.sec = int(now)
        stamp.nanosec = int((now - int(now)) * 1e9)
        
        # 3. Read sensor data and publish
        for i, plate in enumerate(self.force_plates):
            # getValues() retrun [Fx, Fy, Fz]
            values = plate.getValues() 

            # Create msg WrenchStamped 
            msg = WrenchStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = f'force_plate_{i+1}'
            
            msg.wrench.force.x = values[0]
            msg.wrench.force.y = values[1]
            msg.wrench.force.z = -(values[2] - 9.81)
            
            msg.wrench.torque.x = 0.0
            msg.wrench.torque.y = 0.0
            msg.wrench.torque.z = 0.0
            
            # Publish to Topic
            self.publishers[i].publish(msg)

def main(args=None):
    # Standalone execution support
    if not rclpy.ok():
        rclpy.init(args=args)
    
    # In standalone mode, we need a robot instance
    robot = Robot()
    driver = ForcePlateDriver()
    
    # Mocking the webots_node object for standalone compatibility
    class MockWebotsNode:
        def __init__(self, robot):
            self.robot = robot
            
    driver.init(MockWebotsNode(robot), None)
    
    while robot.step(int(robot.getBasicTimeStep())) != -1:
        driver.step()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
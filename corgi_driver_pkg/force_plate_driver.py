"""webots_ros2 plugin for the ForcePlates robot in Corgi_ABAD_force_plate.wbt.

Publishes each plate's Webots force-3d reading as sensor/force_plate_<n>
(geometry_msgs/WrenchStamped), stamped with simulation time -- the same clock
as /clock, so a bag needs no alignment. corgi_data_recorder already subscribes
to these topics.

Frame: the plates are unrotated, so the sensor frame is the world frame (x
forward for a robot spawned facing +x, z up). The force is the one the robot
exerts ON the plate, so a standing leg reads fz < 0 -- the same sign as an
impedance/command fz. The raw Webots value is the opposite sign and also
carries the plate's own weight (it is welded to a static robot, so the reading
is the whole load on that weld -- the reason Eason's 2025 controller subtracted
9.81). Both are removed here. Torques are not measured (force-3d gives none)
and are left at zero.

Check (2026-10-12): standing robot, raw sum over 4 plates ~345 N = 31.04 kg
robot (sum of the proto's masses) + 4 x 1 kg plates.

Layout and numbering match the lab's Vicon/Bertec plates: 1 at +x+y, 2 at
-x+y, 3 at -x-y, 4 at +x-y. A robot facing +x puts A on 1, B on 4, C on 3, D on 2.
"""
import rclpy
from geometry_msgs.msg import WrenchStamped

PLATE_MASS = 1.0  # kg, the plates' Physics mass in Corgi_ABAD_force_plate.wbt
GRAVITY = 9.81


class ForcePlateDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('force_plate_driver')
        self.__plates = []
        for i in range(1, 5):
            dev = self.__robot.getDevice('force_plate_%d' % i)
            if dev is None:
                self.__node.get_logger().error('no device force_plate_%d in the world' % i)
                continue
            dev.enable(timestep)
            pub = self.__node.create_publisher(WrenchStamped, 'sensor/force_plate_%d' % i, 100)
            self.__plates.append((i, dev, pub))
        self.__node.get_logger().info('publishing %d force plates on sensor/force_plate_N' % len(self.__plates))

    def step(self):
        t = self.__robot.getTime()
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        if nsec >= 1000000000:
            sec, nsec = sec + 1, nsec - 1000000000
        for i, dev, pub in self.__plates:
            fx, fy, fz = dev.getValues()[:3]
            msg = WrenchStamped()
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nsec
            msg.header.frame_id = 'force_plate_%d' % i
            msg.wrench.force.x = -fx
            msg.wrench.force.y = -fy
            msg.wrench.force.z = -(fz - PLATE_MASS * GRAVITY)
            pub.publish(msg)

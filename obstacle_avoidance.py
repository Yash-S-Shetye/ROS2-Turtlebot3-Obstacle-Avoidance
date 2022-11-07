# Importing all required ros2 in-built packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

# Defining intial distance to obstacle as 1
distToObstacle = 1

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        qos = QoSProfile(depth=10)

        # Creating a publisher to publish the robot's velocity and a subscriber to subscribe to laser scan messages
        self.sub = self.create_subscription(LaserScan, '/scan', self.obstacle_callback, qos_profile=qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)


    def obstacle_callback(self, msg):
        self.get_logger().info('The distance to obstacle is %s ' % msg.ranges[300])

        move = Twist()

        # Defining conditions for the robot to change direction when it encounters an obstacle
        if msg.ranges[300] > distToObstacle:
            move.linear.x = 0.5
            move.angular.z = 0.0

        if msg.ranges[300] <= distToObstacle:
            move.linear.x = 0.5
            move.angular.z = 1.5 
      
        self.pub.publish(move)

def main(args=None):

    rclpy.init(args=args)

    obstacle_avoidance = ObstacleAvoidance()

    rclpy.spin(obstacle_avoidance)

    obstacle_avoidance.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

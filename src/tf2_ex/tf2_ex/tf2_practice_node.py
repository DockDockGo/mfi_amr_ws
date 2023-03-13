import math
from geometry_msgs.msg import TransformStamped, Twist
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener   
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

from turtlesim.msg import Pose

# from tf2.transformations import quaternion_from_euler
import tf_transformations

class FrameOperator(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_operator')

        # Declare and acquire `turtlename` parameter
        self.turtlename = self.declare_parameter(
          'turtlename', 'turtle1').get_parameter_value().string_value
        
        # Declare and acquire `turtlename` parameter
        self.followername = self.declare_parameter(
          'followername', 'turtle2').get_parameter_value().string_value


        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.turtle1_pose_subscriber = self.create_subscription(
            Pose,
            f'/{self.turtlename}/turtle1/pose',
            self.handle_turtle1_pose,
            1)
        self.turtle1_pose_subscriber  # prevent unused variable warning
        self.turtle2_pose_subscriber = self.create_subscription(
            Pose,
            f'/{self.followername}/turtle1/pose',
            self.handle_turtle2_pose,
            1)
        self.turtle2_pose_subscriber  # prevent unused variable warning

        self.vel_publisher = self.create_publisher(
            Twist, 
            f'/{self.followername}/turtle1/cmd_vel',
            10
        )

        # self.get_logger().info("Transforming from {} to {}".format(self.second_name_, self.first_name_))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_cmd_vel = Twist ()
        self.timer = self.create_timer(0.1, self.timer_callback) #30 Hz = 0.333s

    def timer_callback(self):
        try:
            turtle_transform = self.tf_buffer.lookup_transform(self.followername, 'drone', rclpy.time.Time())



            # self.tf_cmd_vel.linear.x = math.sqrt((turtle_transform.transform.translation.x-0.5) ** 2 + (turtle_transform.transform.translation.y-0.5) ** 2)
            # self.tf_cmd_vel.angular.z = 4 * math.atan2((turtle_transform.transform.translation.y-0.5) , (turtle_transform.transform.translation.x-0.5))
            self.tf_cmd_vel.linear.x = math.sqrt((turtle_transform.transform.translation.x + 0.0) ** 2 + (turtle_transform.transform.translation.y + 0.0) ** 2)
            self.tf_cmd_vel.angular.z = 4 * math.atan2((turtle_transform.transform.translation.y + 0.0) , (turtle_transform.transform.translation.x + 0.0))
            self.vel_publisher.publish(self.tf_cmd_vel)
            # pass
            self.get_logger().info('publishing cmd vel')

        except Exception as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

    def handle_turtle1_pose(self, msg):
        t = TransformStamped()
        t2 = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # t2.header.stamp = self.get_clock().now().to_msg()
        # t2.header.frame_id = self.turtlename
        # t2.child_frame_id = self.followername
        
        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x - 11.08888/2.0
        t.transform.translation.y = msg.y - 11.08888/2.0
        t.transform.translation.z = 0.0
        
        # t2.transform.translation.x = -0.5
        # t2.transform.translation.y = -0.5
        # t2.transform.translation.z = 1.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message

        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]


        # t2.transform.rotation.x = 0.0
        # t2.transform.rotation.y = 0.0
        # t2.transform.rotation.z = 0.0
        # t2.transform.rotation.w = 1.0

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        # self.tf_broadcaster.sendTransform(t2)

    def handle_turtle2_pose(self, msg):
        t2 = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'world'
        t2.child_frame_id = self.followername
        
        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t2.transform.translation.x = msg.x- 11.08888/2.0
        t2.transform.translation.y = msg.y- 11.08888/2.0
        t2.transform.translation.z = 0.0
        
        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message

        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)

        t2.transform.rotation.x = q[0]
        t2.transform.rotation.y = q[1]
        t2.transform.rotation.z = q[2]
        t2.transform.rotation.w = q[3]


        # t2.transform.rotation.x = 0.0
        # t2.transform.rotation.y = 0.0
        # t2.transform.rotation.z = 0.0
        # t2.transform.rotation.w = 1.0

        # Send the transformation
        # self.tf_broadcaster.sendTransform(t)
        self.tf_broadcaster.sendTransform(t2)


def main():
    rclpy.init()
    node = FrameOperator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
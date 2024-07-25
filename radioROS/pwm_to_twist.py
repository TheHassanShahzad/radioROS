import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

class JoystickToTwist(Node):
    def __init__(self):
        super().__init__('joystick_to_twist')
        
        # Declare parameters with default values
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('topic', 'cmd_vel')

        # Get parameter values
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'receiver_data',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(Twist, self.topic, 10)

        self.get_logger().info(f"Joystick to Twist node has started, publishing to topic: {self.topic}")

    def listener_callback(self, msg):
        if len(msg.data) >= 2:
            right_joystick_x = msg.data[0]
            right_joystick_y = msg.data[1]

            # Map joystick values (1000-2000) to speed ranges
            linear_speed = self.map_range(right_joystick_y, 1000, 2000, -self.max_linear_speed, self.max_linear_speed)
            angular_speed = -self.map_range(right_joystick_x, 1000, 2000, -self.max_angular_speed, self.max_angular_speed)

            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed

            self.publisher_.publish(twist)
            self.get_logger().info(f"Published twist: linear.x = {linear_speed}, angular.z = {angular_speed}")

    def map_range(self, value, left_min, left_max, right_min, right_max):
        # Map the value from one range to another
        left_span = left_max - left_min
        right_span = right_max - right_min

        value_scaled = float(value - left_min) / float(left_span)
        return right_min + (value_scaled * right_span)

def main(args=None):
    rclpy.init(args=args)
    joystick_to_twist = JoystickToTwist()

    try:
        rclpy.spin(joystick_to_twist)
    except KeyboardInterrupt:
        pass

    joystick_to_twist.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

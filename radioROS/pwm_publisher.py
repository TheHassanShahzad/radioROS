import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import time
import serial
import sys

baud_rate = 115200

START_BYTE = 0x7E
END_BYTE = 0x7F

class PwmPublisher(Node):
    def __init__(self):
        super().__init__('pwm_publisher')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.publisher_ = self.create_publisher(Int16MultiArray, '/receiver_data', 10)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.get_logger().info("Receiver node started")
        self.get_logger().info(f"Using serial port: {self.serial_port}")

    def publish_data(self, value1, value2, value3, value4, value5, value6):
        msg = Int16MultiArray()
        msg.data = [value1, value2, value3, value4, value5, value6]
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    pwm_publisher = PwmPublisher()

    try:
        with serial.Serial(pwm_publisher.serial_port, baud_rate, timeout=1) as ser:
            while True:
                if ser.read() == bytes([START_BYTE]):
                    data = ser.read(24)
                    if ser.read() == bytes([END_BYTE]):
                        if len(data) == 24:
                            pulse_widths = [
                                int.from_bytes(data[i:i+4], byteorder='big', signed=False)
                                for i in range(0, 24, 4)
                            ]
                            pwm_publisher.publish_data(
                                pulse_widths[0], pulse_widths[1], pulse_widths[2], 
                                pulse_widths[3], pulse_widths[4], pulse_widths[5]
                            )
                            #time.sleep(1)  # Sleep for 1 second between publishes

    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        pwm_publisher.get_logger().error(f"Serial port error: {e}")
    finally:
        pwm_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
import socket
import threading
from std_msgs.msg import Bool
from rclpy.node import Node


class TactileUdpReceiver(Node):

    def __init__(self):
        super().__init__('tactile_udp_receiver', use_global_arguments=False)

        # Declare parameters
        self.declare_parameter('udp_host', '0.0.0.0')
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('topic_name', 'boolean_data')

        # Get parameters
        self.udp_host = self.get_parameter(
            'udp_host').get_parameter_value().string_value
        self.udp_port = self.get_parameter(
            'udp_port').get_parameter_value().integer_value
        self.topic_name = self.get_parameter(
            'topic_name').get_parameter_value().string_value

        # Create publisher
        self.publisher = self.create_publisher(Bool, self.topic_name, 1)

        # Create UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.udp_socket.bind((self.udp_host, self.udp_port))
            self.get_logger().info(
                f'UDP socket bound to {self.udp_host}:{self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to bind UDP socket: {e}')
            return

        # Start UDP listener thread
        self.udp_thread = threading.Thread(target=self.udp_listener,
                                           daemon=True)
        self.running = True
        self.udp_thread.start()

        self.get_logger().info(
            f'Node started. Listening for UDP data on {self.udp_host}:{self.udp_port}'
        )
        self.get_logger().info(
            f'Publishing boolean data to topic: {self.topic_name}')

    def udp_listener(self):
        """Listen for UDP data and publish to ROS2 topic"""
        while self.running:
            try:
                # Receive data from UDP socket (blocking call with timeout)
                self.udp_socket.settimeout(1.0)  # 1 second timeout
                data, addr = self.udp_socket.recvfrom(1024)

                # Decode the received data
                decoded_data = data.decode('utf-8').strip()
                self.get_logger().debug(
                    f'Received data from {addr}: {decoded_data}')

                # Parse boolean value
                boolean_value = self.parse_boolean(decoded_data)

                if boolean_value is not None:
                    # Create and publish ROS2 message
                    msg = Bool()
                    msg.data = boolean_value
                    self.publisher.publish(msg)
                    self.get_logger().info(
                        f'Published boolean: {boolean_value}')
                else:
                    self.get_logger().warning(
                        f'Could not parse boolean from: {decoded_data}')

            except socket.timeout:
                # Timeout is normal, just continue
                continue
            except Exception as e:
                if self.running:  # Only log if we're supposed to be running
                    self.get_logger().error(f'Error in UDP listener: {e}')

    def parse_boolean(self, data_str):
        """Parse various boolean representations from string"""
        data_str = data_str.lower().strip()

        # Try direct boolean conversion
        if data_str in ['true', '1', 'on', 'yes']:
            return True
        elif data_str in ['false', '0', 'off', 'no']:
            return False

        # Try JSON parsing
        try:
            parsed = json.loads(data_str)
            if isinstance(parsed, bool):
                return parsed
            elif isinstance(parsed, (int, float)):
                return bool(parsed)
            elif isinstance(parsed, dict) and 'value' in parsed:
                return bool(parsed['value'])
        except json.JSONDecodeError:
            pass

        # Try integer conversion
        try:
            return bool(int(data_str))
        except ValueError:
            pass

        return None

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.running = False
        if hasattr(self, 'udp_socket'):
            self.udp_socket.close()
        super().destroy_node()


def main():
    rclpy.init()
    udp_receiver_node = TactileUdpReceiver()
    rclpy.spin(udp_receiver_node)
    udp_receiver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

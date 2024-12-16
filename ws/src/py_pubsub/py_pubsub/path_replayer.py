import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialReader, StorageOptions
from geometry_msgs.msg import Twist

class ReplayPath(Node):
    def __init__(self):
        super().__init__('path_replayer')
        self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        self.reader = SequentialReader()

        # Set up rosbag reader
        storage_options = StorageOptions(uri='jetbot_path', storage_id='sqlite3')
        self.reader.open(storage_options)

    def replay(self):
        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()
            if topic == '/jetbot/cmd_vel':
                # Publish the recorded Twist messages
                self.pub.publish(data)
                self.get_logger().info(f"Replaying: {data}")
                rclpy.spin_once(self, timeout_sec=0.05)

def main(args=None):
    rclpy.init(args=args)
    replayer = ReplayPath()
    replayer.replay()
    replayer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

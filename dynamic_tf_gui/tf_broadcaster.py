import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.transform = TransformStamped()

    def timer_callback(self):
        # 設定された変換を送信
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = 'base_link'
        self.transform.child_frame_id = 'livox_frame'
        self.tf_broadcaster.sendTransform(self.transform)

    def update_transform(self, x, y, z, roll, pitch, yaw):
        self.transform.transform.translation.x = x
        self.transform.transform.translation.y = y
        self.transform.transform.translation.z = z
        self.transform.transform.rotation.x = math.sin(roll/2)
        self.transform.transform.rotation.y = math.sin(pitch/2)
        self.transform.transform.rotation.z = math.sin(yaw/2)
        self.transform.transform.rotation.w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
                                              math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

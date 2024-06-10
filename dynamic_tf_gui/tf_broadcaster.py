import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import math


class DynamicTFBroadcaster(Node):
    def __init__(self, frame_id="world", child_frame_id="dynamic_frame"):
        super().__init__("dynamic_tf_broadcaster")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        """ self.declare_parameter("use_sim_time", True) """
        self.use_sim_time = (
            self.get_parameter("use_sim_time").get_parameter_value().bool_value
        )
        self.transform = TransformStamped()
        self.transform.header.frame_id = self.frame_id
        self.transform.child_frame_id = self.child_frame_id

        self.subscription = self.create_subscription(
            PointCloud2,
            "/livox/lidar",  # ここでタイムスタンプ情報を持つ適切なトピック名を使用します
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg):
        self.transform.header.stamp = (
            msg.header.stamp
        )  # 受信したメッセージのタイムスタンプを使用
        self.tf_broadcaster.sendTransform(self.transform)

    def update_transform(self, x, y, z, roll, pitch, yaw):
        self.transform.transform.translation.x = x
        self.transform.transform.translation.y = y
        self.transform.transform.translation.z = z
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        self.transform.transform.rotation.x = qx
        self.transform.transform.rotation.y = qy
        self.transform.transform.rotation.z = qz
        self.transform.transform.rotation.w = qw

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

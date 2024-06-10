import sys
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QSlider,
    QLabel,
    QHBoxLayout,
)
from PyQt5.QtCore import Qt
from tf_broadcaster import DynamicTFBroadcaster
import threading
import rclpy
import argparse


class MainWindow(QMainWindow):
    def __init__(self, frame_id, child_frame_id):
        super().__init__()

        # ROS 2の初期化
        rclpy.init()

        self.node = DynamicTFBroadcaster(frame_id, child_frame_id)
        self.initUI()

        # ROS 2ノードを別スレッドで実行
        threading.Thread(target=self.run_ros_node, daemon=True).start()

    def initUI(self):
        self.setWindowTitle("TF Transform GUI")
        self.setGeometry(500, 500, 400, 300)

        main_layout = QVBoxLayout()

        self.sliders = {}
        for label_text, attr in [
            ("X", "x"),
            ("Y", "y"),
            ("Z", "z"),
            ("Roll", "roll"),
            ("Pitch", "pitch"),
            ("Yaw", "yaw"),
        ]:
            slider_layout = QHBoxLayout()
            label = QLabel(label_text)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-500)
            slider.setMaximum(500)
            slider.setValue(0)
            slider.valueChanged.connect(self.slider_changed)
            slider_layout.addWidget(label)
            slider_layout.addWidget(slider)
            main_layout.addLayout(slider_layout)
            self.sliders[attr] = slider

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def slider_changed(self):
        values = {attr: slider.value() / 100.0 for attr, slider in self.sliders.items()}
        self.node.update_transform(
            values["x"],
            values["y"],
            values["z"],
            values["roll"],
            values["pitch"],
            values["yaw"],
        )

    def run_ros_node(self):
        rclpy.spin(self.node)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dynamic TF Broadcaster GUI")
    parser.add_argument("--frame_id", type=str, default="world", help="Parent frame ID")
    parser.add_argument(
        "--child_frame_id", type=str, default="dynamic_frame", help="Child frame ID"
    )
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = MainWindow(frame_id=args.frame_id, child_frame_id=args.child_frame_id)
    window.show()
    sys.exit(app.exec_())

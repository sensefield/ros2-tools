import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt
from tf_broadcaster import DynamicTFBroadcaster
import threading
import rclpy

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # ROS 2の初期化
        rclpy.init()

        self.node = DynamicTFBroadcaster()
        self.initUI()

        # ROS 2ノードを別スレッドで実行
        threading.Thread(target=self.run_ros_node, daemon=True).start()

    def initUI(self):
        self.setWindowTitle('TF Transform GUI')
        self.setGeometry(100, 100, 400, 300)

        main_layout = QVBoxLayout()

        self.sliders = {}
        for label_text, attr in [('X', 'x'), ('Y', 'y'), ('Z', 'z'), ('Roll', 'roll'), ('Pitch', 'pitch'), ('Yaw', 'yaw')]:
            slider_layout = QHBoxLayout()
            label = QLabel(label_text)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-100)
            slider.setMaximum(100)
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
        values = {attr: slider.value() / 10.0 for attr, slider in self.sliders.items()}
        self.node.update_transform(values['x'], values['y'], values['z'], values['roll'], values['pitch'], values['yaw'])

    def run_ros_node(self):
        rclpy.spin(self.node)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

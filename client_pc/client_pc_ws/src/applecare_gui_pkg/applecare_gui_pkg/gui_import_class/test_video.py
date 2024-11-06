import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import cv2
import numpy as np
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from PyQt5 import uic

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            ROSImage,
            '/camera',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.current_frame = None

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # print(msg)

    def get_frame(self):
        return self.current_frame
    

from_class = uic.loadUiType("/home/zeki/applecare_ws/src/applecare_gui_pkg/applecare_gui_pkg/gui_import_class/untitled.ui")[0]

class VideoGUI(QWidget):
    def __init__(self, node=None):
        super().__init__()
        # self.setupUi(self)
        self.node = node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("ROS Video Stream")
        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.button = QPushButton('Start Video', self)

        self.pixmap = QPixmap()
        self.button.clicked.connect(self.start_video)

        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        layout.addWidget(self.button)
        self.setLayout(layout)

        # Timer to periodically update the QLabel with ROS video data
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)

    def start_video(self):
        # Start the timer to update the QLabel every 100 ms (10 FPS)
        self.timer.start(100)

    def update_image(self):
        frame = self.node.get_frame()
        if frame is not None:
            # Convert the frame to a QImage and display it on the QLabel
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            qimage = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qimage))

            qimage = QImage(frame_rgb,w,h,w*ch, QImage.Format_RGB888)
            self.pixmap = self.pixmap.fromImage(qimage)
            self.pixmap = self.pixmap.scaled(self.video_label.width(),self.video_label.height())

            self.video_label.setPixmap(self.pixmap)

def main(args=None):
    rclpy.init(args=args)

    # Create the ROS 2 node
    video_node = VideoSubscriber()

    # Create the Qt Application
    app = QApplication(sys.argv)

    # Create and display the GUI
    gui = VideoGUI(video_node)
    gui.show()

    # Timer for processing ROS callbacks within the Qt event loop
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(video_node, timeout_sec=0.01))
    ros_timer.start(10)  # Process ROS callbacks every 10ms

    # Run the Qt application
    sys.exit(app.exec_())

    # Shutdown ROS after the application closes
    rclpy.shutdown()

if __name__ == '__main__':
    main()





#---------------------------------------------------------------
# import sys
# from PyQt5.QtWidgets import *
# from PyQt5.QtGui import *
# from PyQt5 import uic


# import rclpy
# from rclpy.node import Node

# from PyQt5.QtCore import QTimer, Qt
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image as ROSImage
# from cv_bridge import CvBridge


# class VideoSubscriber(Node):
#     def __init__(self):
#         super().__init__('video_subscriber')
#         self.subscription = self.create_subscription(
#             ROSImage,
#             '/video',
#             self.listener_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning
#         self.bridge = CvBridge()
#         self.current_frame = None

#     def listener_callback(self, msg):
#         # Convert ROS Image message to OpenCV image
#         self.current_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

#     def get_frame(self):
#         return self.current_frame

# from_class = uic.loadUiType("/home/zeki/applecare_ws/src/applecare_gui_pkg/applecare_gui_pkg/gui_import_class/HelloQt.ui")[0]

# class WindowClass(QMainWindow, from_class) :
#     def __init__(self):
#         super().__init__()
#         self.setupUi(self)

#         self.setWindowTitle("Hello, Qt!")

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     myWindows = WindowClass()
#     myWindows.show()

#     sys.exit(app.exec_())
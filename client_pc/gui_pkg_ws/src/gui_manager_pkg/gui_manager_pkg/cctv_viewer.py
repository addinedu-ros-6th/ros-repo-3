import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5 import QtWidgets

from gui_manager_pkg.gui_import_class.video_subscriber import ImageSubscriber
import cv2
from PyQt5.QtGui import QImage, QPixmap

from PyQt5.QtCore import QTimer, Qt
import rclpy

from_class = uic.loadUiType("src/gui_manager_pkg/gui_manager_pkg/cctv_viewer2.ui")[0]
############################## jpg로 변환 필요!!!!!!!!!!!!!!!!!!!!!! (11.13)
class CctvViewer(QtWidgets.QDialog) :
    def __init__(self,video_node):
        super().__init__()
        # self.setupUi(self)
        # uic.loadUiType("/home/zeki/applecare_ws/src/applecare_gui_pkg/applecare_gui_pkg/cctv_viewer.ui")[0]
        self.video_node = video_node
        
        uic.loadUi('src/gui_manager_pkg/gui_manager_pkg/cctv_viewer2.ui', self)
        self.setWindowTitle("Robot_Viewer")
        self.pixmap = QPixmap()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame1)
        self.timer.timeout.connect(self.update_frame2)
        self.timer.timeout.connect(self.update_frame3)
        self.timer.timeout.connect(self.update_frame4)
        self.timer.start(100)
    def update_frame1(self):
        frame=self.video_node.get_frame1()
        if frame is not None:
            # Convert the frame to a QImage and display it on the QLabel
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            qimage = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.moni1_drive.setPixmap(QPixmap.fromImage(qimage))

            qimage = QImage(frame_rgb,w,h,w*ch, QImage.Format_RGB888)
            self.pixmap = self.pixmap.fromImage(qimage)
            self.pixmap = self.pixmap.scaled(self.moni1_drive.width(),self.moni1_drive.height())

            self.moni1_drive.setPixmap(self.pixmap)

    def update_frame2(self):
        frame=self.video_node.get_frame2()
        if frame is not None:
            # Convert the frame to a QImage and display it on the QLabel
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            qimage = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.moni1_work.setPixmap(QPixmap.fromImage(qimage))

            qimage = QImage(frame_rgb,w,h,w*ch, QImage.Format_RGB888)
            self.pixmap = self.pixmap.fromImage(qimage)
            self.pixmap = self.pixmap.scaled(self.moni1_work.width(),self.moni1_work.height())

            self.moni1_work.setPixmap(self.pixmap)
    def update_frame3(self):
        frame=self.video_node.get_frame3()
        if frame is not None:
            # Convert the frame to a QImage and display it on the QLabel
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            qimage = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.polli1_drive.setPixmap(QPixmap.fromImage(qimage))

            qimage = QImage(frame_rgb,w,h,w*ch, QImage.Format_RGB888)
            self.pixmap = self.pixmap.fromImage(qimage)
            self.pixmap = self.pixmap.scaled(self.polli1_drive.width(),self.polli1_drive.height())

            self.polli1_drive.setPixmap(self.pixmap)
    def update_frame4(self):
        frame=self.video_node.get_frame4()
        if frame is not None:
            # Convert the frame to a QImage and display it on the QLabel
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            qimage = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.polli1_work.setPixmap(QPixmap.fromImage(qimage))

            qimage = QImage(frame_rgb,w,h,w*ch, QImage.Format_RGB888)
            self.pixmap = self.pixmap.fromImage(qimage)
            self.pixmap = self.pixmap.scaled(self.polli1_work.width(),self.polli1_work.height())

            self.polli1_work.setPixmap(self.pixmap)
if __name__ == "__main__":
    rclpy.init()

    # Create the ROS 2 node
    # video_node = ImageSubscriber()
    app = QApplication(sys.argv)
    video_node = ImageSubscriber()
    myWindows = CctvViewer(video_node)
    myWindows.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(video_node, timeout_sec=0.01))
    ros_timer.start(10)

    sys.exit(app.exec_())
    rclpy.shutdown()
# ------------------------------------------------------------------
# from PyQt5 import QtWidgets

# class MainWindow(QtWidgets.QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("메인 윈도우")
        
#         button = QtWidgets.QPushButton("팝업 열기", self)
#         button.clicked.connect(self.show_popup)
        
#     def show_popup(self):
#         popup = PopupDialog()
#         popup.exec_()
        
# class PopupDialog(QtWidgets.QDialog):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("팝업 다이얼로그")
        
#         layout = QtWidgets.QVBoxLayout()
#         layout.addWidget(QtWidgets.QLabel("팝업 내용"))
#         self.setLayout(layout)

# if __name__ == '__main__':
#     app = QtWidgets.QApplication([])
#     window = MainWindow()
#     window.show()
#     app.exec_()
# -------------------------------------------------------------------
# from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
# from PyQt5.QtCore import QTimer
# from PyQt5 import QtWidgets

# class MainWindow(QtWidgets.QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("카운터 예제")
#         self.setGeometry(100, 100, 300, 200)

#         self.counter = 0
        
#         self.label = QLabel("0")
#         self.label.setStyleSheet("font-size: 24pt; font-weight: bold;")
        
#         layout = QVBoxLayout()
#         layout.addWidget(self.label)
        
#         container = QWidget()
#         container.setLayout(layout)
#         self.setCentralWidget(container)

#         self.timer = QTimer(self)
#         self.timer.timeout.connect(self.update_counter)
#         self.timer.start(1000)  # 1000ms = 1초

#         button = QtWidgets.QPushButton("팝업 열기", self)
#         button.clicked.connect(self.show_popup)
        
#     def show_popup(self):
#         popup = PopupDialog()
#         popup.exec_()

#     def update_counter(self):
#         self.counter += 1
#         self.label.setText(str(self.counter))

# class PopupDialog(QtWidgets.QDialog):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("팝업 다이얼로그")
        
#         layout = QtWidgets.QVBoxLayout()
#         layout.addWidget(QtWidgets.QLabel("팝업 내용"))
#         self.setLayout(layout)

# if __name__ == '__main__':
#     app = QApplication([])
#     window = MainWindow()
#     window.show()
#     app.exec_()
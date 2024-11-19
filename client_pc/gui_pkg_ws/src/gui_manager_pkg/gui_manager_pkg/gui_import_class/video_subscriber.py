# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# import cv2
# import numpy as np

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         self.frame1 = None
#         self.frame2 = None

#         # 압축된 이미지 메시지를 구독하는 퍼블리셔 생성
#         self.subscription1 = self.create_subscription(
#             CompressedImage,
#             '/compressed',  # 첫 번째 이미지 토픽 이름
#             self.listener_callback1,
#             5
#         )
        
#         self.subscription2 = self.create_subscription(
#             CompressedImage,
#             '/compressed2',  # 두 번째 이미지 토픽 이름
#             self.listener_callback2,
#             5
#         )
        
#         self.get_logger().info("Image Subscriber 노드가 시작되었습니다.")

#     def listener_callback1(self, msg):
#         # 수신한 압축 이미지를 NumPy 배열로 변환
#         np_array = np.frombuffer(msg.data, np.uint8)
        
#         # 이미지 디코딩
#         self.frame1 = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        
#         # 첫 번째 이미지 표시
#         cv2.imshow("Received Image 1 - /compressed", self.frame1)
#         # cv2.waitKey(1)

#     def listener_callback2(self, msg):
#         # 수신한 압축 이미지를 NumPy 배열로 변환
#         np_array = np.frombuffer(msg.data, np.uint8)
        
#         # 이미지 디코딩
#         self.frame2 = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        
#         # 두 번째 이미지 표시
#         cv2.imshow("Received Image 2 - /compressed2", self.frame2)
#         cv2.waitKey(1)
    
#     def get_frame1(self):
#         return self.frame1

# --------------------------------------------------------------------------
from cv_bridge import CvBridge
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image as ROSImage


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.moni1_drive_sub = self.create_subscription(ROSImage,'/camera1',self.listener_callback1,10)
        self.moni1_work_sub = self.create_subscription(ROSImage,'/camera2',self.listener_callback2,10)
        self.polli1_drive_sub = self.create_subscription(ROSImage,'/camera3',self.listener_callback3,10)
        self.polli1_work_sub = self.create_subscription(ROSImage,'/camera4',self.listener_callback4,10)

        # self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.current_frame1,self.current_frame2,self.current_frame3,self.current_frame4 = None,None,None,None

    def listener_callback1(self, msg):
        # Convert ROS Image message to OpenCV image
        # self.current_frame1 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # print(msg)
        image_array = np.frombuffer(msg.data, np.uint8)
        self.current_frame1 = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    def listener_callback2(self, msg):
        # Convert ROS Image message to OpenCV image
        # self.current_frame2 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        image_array = np.frombuffer(msg.data, np.uint8)
        self.current_frame2 = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    def listener_callback3(self, msg):
        # Convert ROS Image message to OpenCV image
        # self.current_frame3 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        image_array = np.frombuffer(msg.data, np.uint8)
        self.current_frame3 = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    def listener_callback4(self, msg):
        # Convert ROS Image message to OpenCV image
        #self.current_frame4 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

	image_array = np.frombuffer(msg.data, np.uint8)
        self.current_frame4 = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    def get_frame1(self):
        return self.current_frame1
    def get_frame2(self):
        return self.current_frame2
    def get_frame3(self):
        return self.current_frame3
    def get_frame4(self):
        return self.current_frame4
    
from PyQt5.QtCore import QObject, pyqtSignal
from geometry_msgs.msg import PoseWithCovarianceStamped
class amclSubscriber(Node,QObject):
    pose_updated = pyqtSignal(object)
    def __init__(self):
        # super().__init__('amcl_subscriber')
        Node.__init__(self,'amcl_subscriber')
        QObject.__init__(self)
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',self.listener_callback,10)
        self.subscription
        self.pose=None
        
    def listener_callback(self,msg):
        self.pose = msg
        print("$$$$$$")
        print(self.pose)
        print("$$$$$")
        self.pose_updated.emit(self.pose)  # 시그널 발생
        # print(self.pose.pose.pose.position.x)
    
    def get_pose(self):
        return self.pose
    
if __name__ == "__main__":
    rclpy.init()
    amcl = amclSubscriber()
    rclpy.spin(amcl)
    amcl.destroy_node()
    rclpy.shutdown()

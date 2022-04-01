import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import time
import cv2

class imageSubscriber(Node):
    
    def __init__(self):
        super().__init__('imageSubscriber')
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth = 5
        )
        self.subscription_image = self.create_subscription(
            Image,
            'camera/image_raw',
            self.callback,
            qos_profile=qos_profile
        )
        self.detector = cv2.QRCodeDetector()
        self.prevData = 20
        

    def callback(self, data):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().log(e)
        self.findQr(cv_image)

        
    def findQr(self, image):
        data,bbox,rectifiedImage = self.detector.detectAndDecode(image)
        if (data != "" and data != self.prevData):
            print(str(data))
            self.prevData = data
       
        


    

def main():
    rclpy.init(args=None)
    rp = imageSubscriber()
    rclpy.spin(rp)

if __name__ == '__main__':
    main()
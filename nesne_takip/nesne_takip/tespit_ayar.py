import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

def nothing(x):
    pass
#Görüntüyü küçültme
def goruntu_kucult(img, yuzde=0.3):

    width = int(img.shape[1] * yuzde)
    height = int(img.shape[0] * yuzde)
    boyut = (width, height)
    return cv2.resize(img, boyut, interpolation=cv2.INTER_AREA)

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10)
        
        self.publisher_ = self.create_publisher(Point, '/vision/target_info', 10)
        
        self.bridge = CvBridge()
        
        cv2.namedWindow("Trackbar")
        cv2.resizeWindow("Trackbar", 400, 300) #Sabit Trackbar penceresi
        
        cv2.createTrackbar("Min H", "Trackbar", 0, 179, nothing)
        cv2.createTrackbar("Min S", "Trackbar", 0, 255, nothing)
        cv2.createTrackbar("Min V", "Trackbar", 0, 255, nothing)

        cv2.createTrackbar("Max H", "Trackbar", 179, 179, nothing) 
        cv2.createTrackbar("Max S", "Trackbar", 255, 255, nothing)
        cv2.createTrackbar("Max V", "Trackbar", 255, 255, nothing)
        
        self.get_logger().info("Vision Node (Dikdortgen Takibi - Waffle) Baslatildi.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Görüntü dönüşüm hatası: {e}")
            return

        cv_image = goruntu_kucult(cv_image, yuzde=0.3)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        alt_h = cv2.getTrackbarPos("Min H", "Trackbar")
        alt_s = cv2.getTrackbarPos("Min S", "Trackbar")
        alt_v = cv2.getTrackbarPos("Min V", "Trackbar")
        
        ust_h = cv2.getTrackbarPos("Max H", "Trackbar")
        ust_s = cv2.getTrackbarPos("Max S", "Trackbar")
        ust_v = cv2.getTrackbarPos("Max V", "Trackbar")
        
        az = np.array([alt_h, alt_s, alt_v])
        cok = np.array([ust_h, ust_s, ust_v])
        
        mask = cv2.inRange(hsv, az, cok)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        target_msg = Point()
        height, width, _ = cv_image.shape

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            if area > 200:
                x, y, w, h = cv2.boundingRect(c)
                
                cx = x + int(w / 2)
                cy = y + int(h / 2)
                
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (100, 100, 100), 2)
                cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                
                cx_norm = (cx - (width / 2)) / (width / 2)

                target_msg.x = float(cx_norm) 
                target_msg.y = float(area)    
                target_msg.z = 1.0            
                
            else:
                target_msg.z = 0.0
        else:
            target_msg.z = 0.0

        self.publisher_.publish(target_msg)
        
        cv2.imshow("Maske Ayari", mask)
        cv2.imshow("Waffle Kamera", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
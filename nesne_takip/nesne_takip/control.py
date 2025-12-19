import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # --- ABONELİKLER ---
        self.vision_sub = self.create_subscription(
            Point, '/vision/target_info', self.vision_callback, 10)
            
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile)
            
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # --- AYARLAR ---
        self.Kp = 1.0            # Dönüş hassasiyeti
        self.linear_speed = 0.2  # İlerleme hızı
        
        #DURMA MESAFESİ (Metre)
        self.stop_distance = 0.75 # 75 cm
        
        self.deadband = 0.05     # Merkezleme toleransı
        self.search_angular_speed = 0.15 # Hedef ararken dönme hızı
        
        # --- DEĞİŞKENLER ---
        self.current_distance = 10.0 # Başlangıçta önü boş varsay
        self.last_msg_time = self.get_clock().now()
        
        self.target_visible = False
        self.target_error_x = 0.0
        self.detection_counter = 0 
        
        self.get_logger().info("Mod: Büyüklük Fark Etmeksizin Renge Git ve 75cm Kala Dur.")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def scan_callback(self, msg):
        # Lidar verisi işleme (Sonsuz/Inf değerlerini temizleme)
        ranges = msg.ranges
        front_ranges = ranges[0:10] + ranges[-10:]
        
        clean_ranges = []
        for r in front_ranges:
            if math.isinf(r):
                clean_ranges.append(10.0) # Sonsuz ise 10 metre kabul et
            elif r > 0.05:
                clean_ranges.append(r)
        
        if len(clean_ranges) > 0:
            self.current_distance = min(clean_ranges)
        else:
            self.current_distance = 10.0

    def vision_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        
        is_detected = (msg.z > 0.5)
        
        if is_detected:
            self.detection_counter += 1
        else:
            self.detection_counter = 0 
            
        # 2 kere üst üste görse yeter (Hızlı tepki versin)
        if self.detection_counter > 2:
            self.target_visible = True
        else:
            self.target_visible = False
            
        self.target_error_x = msg.x

    def control_loop(self):
        twist = Twist()
        now = self.get_clock().now()

        # Güvenlik: Veri gelmiyorsa dur
        time_since_msg = (now - self.last_msg_time).nanoseconds / 1e9
        if time_since_msg > 1.0:
            self.stop_robot()
            print("Kamera Bağlantısı Yok! - STOP", end='\r')
            return
        
        if self.target_visible:
            
            # A) YÖNELME (Rotasyon)
            if abs(self.target_error_x) > self.deadband:
                twist.angular.z = -self.Kp * self.target_error_x
            
            # B) İLERLEME VE DURMA (Lidar Mesafe Kontrolü)
            if self.current_distance > self.stop_distance:
                # Mesafe 75 cm'den büyükse İLERLE
                twist.linear.x = self.linear_speed
                durum = f"GIDILIYOR (Mesafe: {self.current_distance:.2f}m)"
            else:
                # Mesafe 75 cm veya daha azsa DUR
                twist.linear.x = 0.0
                durum = "HEDEFE VARILDI (STOP)"
                
        else:
            # HEDEF YOK ===
            # Olduğu yerde dönerek ara
            twist.linear.x = 0.0
            twist.angular.z = self.search_angular_speed
            durum = "ARANIYOR (Dönülüyor...)"

        self.publisher_.publish(twist)
        print(f"[{durum}]", end='\r')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
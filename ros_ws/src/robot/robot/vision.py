import math

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from interfaces.msg import Detect
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.bridge = CvBridge()
        self.model = YOLO("src/robot/robot/weights.pt")

        self.image_sub = self.create_subscription(CompressedImage, 'image_raw/compressed', self.image_callback, 1)

        # self.cube_image_pub = self.create_publisher(CompressedImage, 'processed/compressed', 1)

        self.detect_pub = self.create_publisher(Detect, 'detect', 1)

        self.get_logger().info("Starting vision node")
        
    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame)

        closest = [Point(), Point(), Point()]

        for i in range(3):
            closest[i].x = 0.0
            closest[i].y = float('inf')
            closest[i].z = 0.0

        VFOV = 30*(math.pi/180)
        HFOV = 1.04*VFOV*(640/480)
        
        # Iterate over the detections to extract bounding box details
        for i in range(len(results[0].boxes)):
            box = results[0].boxes[i]
            color = int(box.cls)

            x_min, y_min, x_max, y_max = box.xyxy[0]
            height = y_max - y_min

            distance = 480/(height*math.tan(VFOV/2))

            if distance < closest[color].y:
                closest[color].y = float(distance)
                x_pixels = (x_min + x_max - 640) / 2.0
                theta = x_pixels * (HFOV/640)

                L = 2

                x_dist = float(L*distance*math.tan(theta)/2)
                closest[color].x = x_dist
                scalar = 1 + 0.04 * abs(x_dist) / 5.8

                closest[color].x *= scalar
                closest[color].y *= scalar

                # returns one if not touching top/bottom edges
                closest[color].z = float(1.0 if (y_min > 0 and y_max < 480-1) else 0.0)

        detect = Detect()
        detect.green = closest[0]
        detect.red = closest[1]
        detect.sphere = closest[2]

        # Publish the positions of the blocks
        self.detect_pub.publish(detect)
        
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down vision node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
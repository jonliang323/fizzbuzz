import rclpy
from cv_bridge import CvBridge
from interfaces.msg import Box, Detect
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.bridge = CvBridge()
        self.model = YOLO("src/robot/robot/weights.pt")

        self.camera_sub = self.create_subscription(CompressedImage, 'image_raw/compressed', self.camera_callback, 1)

        # self.processed_pub = self.create_publisher(CompressedImage, 'processed/compressed', 1)

        self.detect_pub = self.create_publisher(Detect, 'detect', 1)

        self.get_logger().info("Starting vision node")
        
    def camera_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # chop off top half of image
        frame[:240, :] = 0

        # run through model
        results = self.model(frame)

        widest = [Box(), Box()]

        MIN_WIDTH = 10
        EDGE_TOLERANCE = 2

        # initialize boxes
        for i in range(2):
            widest[i].x = 0
            widest[i].y = 0
            widest[i].width = 0
            widest[i].height = 0

        for i in range(len(results[0].boxes)):
            # get box data
            box = results[0].boxes[i]
            color = int(box.cls)

            # ignore spheres
            if color == 2:
                continue

            # check if widest
            x_min, y_min, x_max, y_max = box.xyxy[0]
            width = x_max - x_min
            if width <= widest[color].width:
                continue

            # minimum block width
            if width < MIN_WIDTH:
                continue

            # is touching edges
            if x_min < EDGE_TOLERANCE or x_max > (640-1-EDGE_TOLERANCE) or y_max > (480-1-EDGE_TOLERANCE):
                continue

            # set box variables
            widest[color].width = int(width)
            widest[color].height = int(y_max - y_min)
            widest[color].x = int((x_min+x_max)/2) - 320
            widest[color].y = int((y_min+y_max)/2)
        
        detect = Detect()
        detect.green = widest[1]
        detect.red = widest[0]

        # Publish the positions of the blocks
        self.detect_pub.publish(detect)

        # Publish processed image
        # compressed_img_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        # self.processed_pub.publish(compressed_img_msg)
        
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
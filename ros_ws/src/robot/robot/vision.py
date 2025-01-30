import rclpy
import cv2
from cv_bridge import CvBridge
from interfaces.msg import Box, Detect
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.model = YOLO("src/robot/robot/weights.pt")

        self.camera_sub = self.create_subscription(CompressedImage, 'image_raw/compressed', self.camera_callback, 1)

        # self.processed_pub = self.create_publisher(CompressedImage, 'processed/compressed', 1)

        self.detect_pub = self.create_publisher(Detect, 'detect', 1)

        self.get_logger().info("Starting vision node")
        self.num_saved = 1
        self.save = True
        self.COLOR_VALUES = [(0,255,0), (0,0,255), (255,0,0)]
        
    def camera_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #preprocess to get rid of pixels above blue tape
        #Blue processing
        blue_mask = cv2.inRange( #output b and w mask, w pixels are blue
            hsv,
            np.array([int(190/2),int(0.5*255),int(0.1*255)]), #min hsv blue 228 60 20
            np.array([int(240/2),int(8*255),int(0.5*255)]), #max hsv blue
        )
        blue_temp = np.where(blue_mask == 255) #tuple of row col lists
        if blue_temp == []:
            blue_locs = [(0,0,'b')] #default, would clear top left corner if no blue detected
        else:
            blue_locs = list(zip(blue_temp[0],blue_temp[1],['b']*len(blue_temp[0])))
        #Orange processing
        # orange_mask = cv2.inRange( #output b and w mask, w pixels are orange
        #     hsv,
        #     np.array([int(20/2),int(0.82*255),int(0.35*255)]), #min hsv orange
        #     np.array([int(32/2),int(1*255),int(0.7*255)]), #max hsv orange
        # )
        # orange_temp = np.where(orange_mask == 255) #tuple of row col lists
        # if orange_temp == []:
        #     orange_locs = []
        # else:
        #     orange_locs = list(zip(orange_temp[0],orange_temp[1],['o']*len(orange_temp[0])))
        orange_locs = []
        
        col = 0
        all_locs = sorted(blue_locs + orange_locs, key=lambda x:(x[1],x[0])) #sort (row, col, color) locs by col, then row
        #finds maximum orange,blue row values associated to each col, then clears image above that row
        for i in range(len(all_locs)):
            if i == len(all_locs) - 1 or all_locs[i+1][1] != col: #found transition to next col, or last col
                max_masked_loc = all_locs[i]
                # if max_masked_loc[2] == 'b':
                frame[:max_masked_loc[0],col] = (0,0,0)
                # else: # == 'o' for visual debugging
                #     frame[:max_masked_loc[0],col] = (255,255,255)
                col += 1

        # chop off top half of image
        # frame = full_frame
        # frame[:240, :] = 0

        # run through model
        results = self.model(frame)
        boxes = results[0].boxes

        widest = [Box(), Box()]

        MIN_WIDTH = 10
        EDGE_TOLERANCE = 2

        # initialize boxes
        for i in range(2):
            widest[i].x = 0
            widest[i].y = 0
            widest[i].width = 0
            widest[i].height = 0

        for i in range(len(boxes)):
            # get box data
            box = boxes[i]

            # HS FILTER VARIABLES
            coords = boxes.xyxy[i].round()
            x1,x2 = int(coords[0]), int(coords[2])
            y1,y2 = int(coords[1]), int(coords[3])

            ############# HS FILTER #############
            hsv_box = hsv[y1:y2, x1:x2]
            avg_h, avg_s = np.mean(hsv_box, axis=(0,1))[0:2]

            is_red = avg_h <= 15 or avg_h >= 165
            is_green = avg_h >= 35 and avg_h <= 80

            if avg_s <= 12 and not (is_red or is_green):
                continue
            ############# HS FILTER #############

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

            if self.save and self.num_saved%10 == 0:
                cv2.rectangle(frame,(int(x_min),int(y_min)),(int(x_max),int(y_max)),self.COLOR_VALUES[color],2) #output bounding box

        if self.save:
            if self.num_saved%8 == 0:
                cv2.imwrite(f'./pics/{self.num_saved}.jpg', frame)
            self.num_saved += 1
            if self.num_saved > 200:
                self.save = False
        
        detect = Detect()
        detect.green = widest[0]
        detect.red = widest[1]

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

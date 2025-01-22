import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
from rclpy.clock import Clock

class LiveLabelNode(Node):

    def __init__(self):
        super().__init__('live_label_subscriber')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)
        self.clock = Clock()
        self.prev_time = self.clock.now()
        self.count = 0
        
    def image_callback(self, msg: CompressedImage):
        if (self.clock.now() - self.prev_time).nanoseconds/1e9 > 8:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            #preprocess to get rid of pixels above blue tape
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange( #output b and w mask where white pixels are in range
                hsv,
                np.array([int(190/2),int(0.5*255),int(0.1*255)]), #min hsv blue 228 60 20
                np.array([int(240/2),int(8*255),int(0.5*255)]), #max hsv blue
            )
            temp_bl = np.where(mask == 255) #tuple of row col lists
            if temp_bl == []:
                blue_locs = [(0,0)] #default, would clear top left corner if no blue detected
            else:
                blue_locs = sorted(list(zip(temp_bl[0],temp_bl[1])), key=lambda x:(x[1],x[0])) #sorted locs by col, then row
            col = 0
            #finds maximum blue row valuesassociated to each col, then clears image above that row
            for i in range(len(blue_locs)):
                if i == len(blue_locs) - 1 or blue_locs[i+1][1] != col: #found transition
                    frame[:blue_locs[i][0],col] = (0,0,0)
                    col += 1
            ##
            # Now, feed into model for classification and bounding box
            ##
            model = YOLO("image_processing/image_processing/yolo_weights/best.pt")
            results = model(frame, imgsz=(480,640))
            boxes = results[0].boxes #1 image processed
            coords = boxes.xyxy
            obj_type = boxes.cls
            for i in range(len(coords)):
                x1, y1 = int(coords[i][0]), int(coords[i][1])
                x2, y2 = int(coords[i][2]), int(coords[i][3])
                if obj_type[i] == 0: #green
                    color = (0,255,0)
                elif obj_type[i] == 1: #red
                    color = (0,0,255)
                else:
                    color = (255,0,0)
                cv2.rectangle(frame,(x1,y1),(x2,y2),color,2) #output bounding box
            
            print(f"{cv2.imwrite(f"./pics/{self.count}.jpg", frame)} {self.count}")
            self.count += 1
            self.prev_time = self.clock.now()
        else:
            print(f"Time: {int((self.clock.now()-self.prev_time).nanoseconds/1e9)}")
    
def main(args=None):
    rclpy.init()
    node = LiveLabelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

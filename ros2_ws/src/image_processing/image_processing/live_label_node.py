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
            orange_mask = cv2.inRange( #output b and w mask, w pixels are orange
                hsv,
                np.array([int(190/2),int(0.5*255),int(0.1*255)]), #min hsv orange
                np.array([int(240/2),int(8*255),int(0.5*255)]), #max hsv orange
            )
            orange_temp = np.where(orange_mask == 255) #tuple of row col lists
            if orange_temp == []:
                orange_locs = []
            else:
                orange_locs = list(zip(orange_temp[0],orange_temp[1],['o']*len(orange_temp[0])))
            
            all_locs = sorted(blue_locs + orange_locs, key=lambda x:(x[1],x[0])) #sort (row, col, color) locs by col, then row
            #find average pixel boundary height (for orange + blue combined)
            avg_boundary_pixel = 0
            otr = 0.0
            if len(all_locs) > 0:
                col = 0
                #finds maximum orange,blue row values associated to each col, then clears image above that row
                for i in range(len(all_locs)):
                    if i == len(all_locs) - 1 or all_locs[i+1][1] != col: #found transition to next col, or last col
                        max_masked_loc = all_locs[i]
                        if max_masked_loc[2] == 'b':
                            frame[:max_masked_loc[0],col] = (0,0,0)
                        else: # == 'o' for visual debugging
                            frame[:max_masked_loc[0],col] = (255,255,255)
                        avg_boundary_pixel += max_masked_loc[0]
                        col += 1
                avg_boundary_pixel = avg_boundary_pixel//col
                otr = len(orange_locs)/len(all_locs)
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

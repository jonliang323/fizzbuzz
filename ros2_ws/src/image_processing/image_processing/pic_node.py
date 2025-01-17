import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

class PicNode(Node):

    def __init__(self):
        super().__init__('pic_subscriber')
        self.bridge = CvBridge()
        self.clock = Clock()
        self.prev_time = self.clock.now()
        self.count = 0
        self.pic_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.pic_callback, 10)
        
    def pic_callback(self, msg: CompressedImage):
        if (self.clock.now() - self.prev_time).nanoseconds/1e9 > 3:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            #preprocess to get rid of pixels above blue line
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
            #finds maximum blue row values associated to each col, then clears image above that row
            for i in range(len(blue_locs)):
                if i == len(blue_locs) - 1 or blue_locs[i+1][1] != col: #found transition
                    frame[:blue_locs[i][0],col] = (0,0,0)
                    col += 1
            print(f"{cv2.imwrite(f"./pics/{self.count}.jpg", frame)} {self.count}")
            self.count += 1
            self.prev_time = self.clock.now()
        
    
def main(args=None):
    rclpy.init()
    node = PicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
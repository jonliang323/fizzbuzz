import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from image_processing_interfaces.msg import CubeTracking 
import cv2
from cv_bridge import CvBridge
import numpy as np

class CubeDetectNode(Node):

    def __init__(self):
        super().__init__('cube_detect_subscriber')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)

        # self.cube_maskBW_pub = self.create_publisher(CompressedImage, "cube_maskBW/compressed", 10)
        self.cube_box_pub = self.create_publisher(CompressedImage, "cube_box/compressed", 10)
        # self.cube_maskC_pub = self.create_publisher(CompressedImage, "cube_maskC/compressed", 10)
        self.location_pub = self.create_publisher(CubeTracking, "cube_location_info", 10)
        
    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        frame = cv2.medianBlur(frame, 51)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # create B and W mask with range
        mask = cv2.inRange( #output 1 b and w mask
            hsv,
            np.array([30,20,20]), #min hsv published ref ([70,50,50], [90,255,255])
            np.array([80,255,255]), #max hsv
        )

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        frame_box = np.copy(frame)
        x,y,w,h = None,None,None,None
        if len(contours) != 0:
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(frame_box,(x,y),(x+w,y+h),(0,0,255),2) #output 2 bounding box
        
        
        #output 3 mask on image
        # mask_3c = cv2.merge([mask,mask,mask])
        # white_im = np.full_like(frame, (255,255,255))
        # frame = np.where(mask_3c == (255,255,255), white_im, frame)

        #output 4 print distance to cube
        
        if x is None:
            cube_center_x = -1
            cube_dist = -1.0
        else:
            y_px = h #from bounding box
            h_px = frame.shape[0]
            h_mm = 1 #mm, param, to adjust
            f = 2 #mm, param, to adjust
            k = 1 #linear offset
            L = 2*25.4 #mm
            cube_dist = round(L*f*h_px/(h_mm*y_px)/10 - k,2) #cm
            cube_center_x = x + w//2
        print(x,w)


        # cube_maskBW_msg = self.bridge.cv2_to_compressed_imgmsg(mask) #-> compress for transport
        # self.cube_maskBW_pub.publish(cube_maskBW_msg)

        cube_box_msg = self.bridge.cv2_to_compressed_imgmsg(frame_box)
        self.cube_box_pub.publish(cube_box_msg)

        # cube_maskC_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        # self.cube_maskC_pub.publish(cube_maskC_msg)
        cube_info_msg = CubeTracking()
        cube_info_msg.x_center = int(cube_center_x)
        cube_info_msg.distance = float(cube_dist)
        self.location_pub.publish(cube_info_msg)
    
def main(args=None):
    rclpy.init()
    node = CubeDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

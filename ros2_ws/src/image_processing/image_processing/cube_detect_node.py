import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from image_processing_interfaces.msg import CubeTracking 
import cv2
from cv_bridge import CvBridge
import numpy as np
import yolo

class CubeDetectNode(Node):

    def __init__(self):
        super().__init__('cube_detect_subscriber')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)
        self.location_pub = self.create_publisher(CubeTracking, "cube_location_info", 10)
        self.box_map_pub = self.create_publisher(CompressedImage, "box_map", 10)

        self.H_MM = 1 #mm, param, to adjust
        self.F = 2 #mm, param, to adjust
        self.K = 1 #linear offset
        self.L = 2*25.4 #mm size of cube
        
    def image_callback(self, msg: CompressedImage):
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
        x_center_list = []
        distance_list = []
        obj_type_list = []

        #print(f'results detected -------------------- :\n{boxes}')

        for i in range(len(boxes)):
            obj_type = int(boxes.cls[i])
            coords = boxes.xyxy[i]
            x_center = int((coords[0] + coords[2])/2)
            # distance to cube
            y_px = int(coords[1] - coords[3]) #from bounding box
            h_px = frame.shape[0]
            distance = round(self.L*self.F*h_px/(self.H_MM*y_px)/10 - self.K,2) #cm

            x_center_list.append(x_center)
            distance_list.append(distance)
            obj_type_list.append(obj_type)

        #print(f"{len(boxes)} objects detected on the screen")

        cube_info_msg = CubeTracking()
        # print(f'x_center: {x_center_list} \n dist: {distance_list} \n obj: {obj_type_list}')
        cube_info_msg.x_centers = x_center_list
        cube_info_msg.distances = distance_list
        cube_info_msg.obj_types = obj_type_list
        self.location_pub.publish(cube_info_msg)


    
def main(args=None):
    rclpy.init()
    node = CubeDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

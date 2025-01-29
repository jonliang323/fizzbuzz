import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from image_processing_interfaces.msg import CubeTracking, WallInfo
from std_msgs.msg import Bool, Int16
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO

class CubeDetectNode(Node):

    def __init__(self):
        super().__init__('cube_detect_subscriber')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)
        self.cv_sub = self.create_subscription(Bool, "scan_blocks", self.cv_callback, 10)
        self.cube_info_pub = self.create_publisher(CubeTracking, "cube_info", 10)
        self.wall_info_pub = self.create_publisher(WallInfo, "wall_info", 10)

        self.H_MM = 1 #mm, param, to adjust
        self.F = 2 #mm, param, to adjust
        self.K = 1 #linear offset
        self.L = 2*25.4 #mm size of cube

        self.frame = None
        self.model = YOLO("image_processing/image_processing/yolo_weights/best.pt")
        
    def image_callback(self, msg: CompressedImage):
        self.frame = msg

    def cv_callback(self, scan_blocks: Bool):
        #gets last published frame from memory if not default
        if not self.frame:
            self.get_logger().info('Waiting on frame to be published by camera')
            return
        cur_frame = self.bridge.compressed_imgmsg_to_cv2(self.frame, "bgr8")
        #preprocess to get rid of pixels above blue tape
        hsv = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2HSV)
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
                        cur_frame[:max_masked_loc[0],col] = (0,0,0)
                    else: # == 'o' for visual debugging
                        cur_frame[:max_masked_loc[0],col] = (255,255,255)
                    avg_boundary_pixel += max_masked_loc[0]
                    col += 1
            avg_boundary_pixel = avg_boundary_pixel/col
            otr = len(orange_locs)/len(all_locs)

        min_o_height = 0
        max_o_height = 999
        for loc in orange_locs:
            if loc[0] < min_o_height:
                min_o_height = loc[0]
            if loc[0] > max_o_height:
                max_o_height = loc[0]
        orange_height_range = max_o_height - min_o_height


        wall_info_msg = WallInfo()
        wall_info_msg.orange_tape_ratio = float(otr)
        wall_info_msg.avg_height = int(avg_boundary_pixel)
        wall_info_msg.height_range = int(orange_height_range)
        self.wall_info_pub.publish(wall_info_msg)

        if scan_blocks: #if we want to look for blocks too
            ##
            # Now, feed into model for classification and bounding box
            ##
            results = self.model.predict(cur_frame, imgsz=(480,640),verbose=False)
            boxes = results[0].boxes #1 image processed
            size_list = []
            obj_type_list = []
            x_center_list = []
            y_center_list = []

            block_pixels = 0
            covered = []
            #print(f'results detected -------------------- :\n{boxes}')
            for i in range(len(boxes)):
                obj_type = int(boxes.cls[i])
                coords = boxes.xyxy[i].round()
                x1,x2 = int(coords[0]), int(coords[2])
                y1,y2 = int(coords[1]), int(coords[3])
                w = x2-x1
                h = y2-y1
                size = w*h
                new_box_overlap = 0
                for entry in covered: #finds overlap between this box and all others
                    #overlap += self.find_box_overlap(entry,(x1,x2,y1,y2))
                    xE1, xE2, yE1, yE2 = entry[0], entry[1], entry[2], entry[3]
                    x_overlap, y_overlap = 0, 0
                    if x1 > xE1 and x1 < xE2 or x2 > xE1 and x2 < xE2:
                        x_list = sorted([x1,x2,xE1,xE2])
                        x_overlap = x_list[2]-x_list[1]
                    if y1 > yE1 and y1 < yE2 or y2 > yE1 and y2 < yE2:
                        y_list = sorted([y1,y2,yE1,yE2])
                        y_overlap = y_list[2]-y_list[1]
                    new_box_overlap += x_overlap*y_overlap
                block_pixels += (size-new_box_overlap)
                covered.append((x1,x2,y1,y2))

                obj_type_list.append(obj_type)
                size_list.append(size)
                x_center_list.append((x1+x2)//2)
                y_center_list.append((y1+y2)//2)

            cube_info_msg = CubeTracking()
            cube_info_msg.sizes = size_list
            cube_info_msg.obj_types = obj_type_list
            cube_info_msg.x_centers = x_center_list
            cube_info_msg.block_pixels = block_pixels
            cube_info_msg.y_centers = y_center_list
            self.cube_info_pub.publish(cube_info_msg)
    
def main(args=None):
    rclpy.init()
    node = CubeDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

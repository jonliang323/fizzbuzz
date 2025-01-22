# import numpy as np
# import cv2
# import os

# try:
#     # List all files in the folder
#     folder_path = './pics'
#     files = os.listdir(folder_path)

#     # Loop through the files
#     for i, file_name in enumerate(files):
#         # Build the new file name
#         old_file_path = os.path.join(folder_path, file_name)

#         # Create the new file name
#         new_file_name = f"{i}.png"
#         new_file_path = os.path.join(folder_path, new_file_name)

#         # Rename the file
#         os.rename(old_file_path, new_file_path)

# except Exception as e:
#     print(f"An error occurred: {e}")

#functionality dos
# try:
#     # List all files in the folder
#     folder_path = './pics'
#     files = os.listdir(folder_path)

#     # Loop through the files
#     for i, file_name in enumerate(files):
#         if np.random.rand() > 0.6: #brightness or spatial augmentation with 40% chance
#             full_path = os.path.join(folder_path, file_name)
#             img = cv2.imread(full_path)
#             a = np.random.rand()
            #3 possibile transformations
            # if a > 0.66: #brightness
            #     factor = np.random.rand()*2-1 #brightness
            #     img = cv2.convertScaleAbs(img, alpha=1, beta=factor*40)
            # elif a > 0.33: #contrast
            #     factor = np.random.rand()+0.5 #contrast
            #     img = cv2.convertScaleAbs(img, alpha=factor, beta=0)
            # else: #blur
            #     size = np.random.randint(5,16)*2+1 #odd kernel, 11 to 31
            #     img = cv2.GaussianBlur(img, (size,size), 0)
            
            #4 possibile transformations, ran independently
            # if a > 1: #flip
            #     img = cv2.flip(img, 1)
            # elif a > 8: #crop, resize
            #     crop_x = np.random.randint(200, 641)
            #     crop_y = np.random.randint(200, 481)
            #     x = np.random.randint(0,640 - crop_x + 1)
            #     y = np.random.randint(0,480 - crop_y + 1)
            #     img = img[y:y+crop_y,x:x+crop_x]
            #     img = cv2.resize(img,(640,480))
            # elif a > 0.2: #rotate
            #     #center = (320, 240)
            #     angle = np.random.randint(-15,15)
            #     rot_matrix = cv2.getRotationMatrix2D((320,240),angle,scale=1)
            #     img = cv2.warpAffine(img,rot_matrix,(640,480))
            # else: #shift
            #     (shift_x, shift_y) = np.random.randint(50,201,size=2)*np.random.choice([-1,1],size=2)
            #     t_matrix = np.float32([[1,0,shift_x],[0,1,shift_y]])
            #     img = cv2.warpAffine(img, t_matrix, (640,480))
            
            # cv2.imwrite(f'{folder_path}/{file_name[:-4]}b.jpg', img)

# except Exception as e:
#     print(f"An error occurred: {e}")
import numpy as np
from ultralytics import YOLO
model = YOLO("image_processing/image_processing/yolo_weights/best.pt")
source = np.random.randint(low=0, high=255, size=(640, 640, 3), dtype="uint8")
results = model(source, image_size=640)
results[0].show()
print(results[0].boxes[0].xyxy)
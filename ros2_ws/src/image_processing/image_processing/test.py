import numpy as np
import cv2
import os

# try:
#     # List all files in the folder
#     folder_path = './pics'
#     files = os.listdir(folder_path)

#     # Loop through the files
#     for i, file_name in enumerate(files):
#         # Build the new file name
#         old_file_path = os.path.join(folder_path, file_name)

#         # Create the new file name
#         new_file_name = f"{i}.jpg"
#         new_file_path = os.path.join(folder_path, new_file_name)

#         # Rename the file
#         os.rename(old_file_path, new_file_path)

# except Exception as e:
#     print(f"An error occurred: {e}")

try:
    # List all files in the folder
    folder_path = './pics'
    files = os.listdir(folder_path)

    # Loop through the files
    for i, file_name in enumerate(files):
        if np.random() > 0.6: #brightness or spatial augmentation with 40% chance
            full_path = os.path.join(folder_path, file_name)
            img = cv2.imread(full_path)
            alpha = np.random()
            #3 possibile transformations
            if alpha > 0.66:
                sign = (int(100*alpha)-66-33)//33
                img = cv2.convertScaleAbs(img, alpha=1, beta=sign*20)
            elif alpha > 0.33:
                factor = (int(100*alpha)-33)/33 #decrease contrast
                img = cv2.convertScaleAbs(img, alpha=factor, beta=0)
            else:
                size = int((int(100*alpha)/33)*10)*2+1 #odd kernel
                img = cv2.GaussianBlur(img, (size,size), 0)
            
            #4 possibile transformations, ran independently
            if alpha > 0.66:
                sign = (int(100*alpha)-66-33)//33
                img = cv2.convertScaleAbs(img, alpha=1, beta=sign*20)
            elif alpha > 0.33:
                factor = (int(100*alpha)-33)/33 #decrease contrast
                img = cv2.convertScaleAbs(img, alpha=factor, beta=0)
            else:
                size = int((int(100*alpha)/33)*10)*2+1 #odd kernel
                img = cv2.GaussianBlur(img, (size,size), 0)
        

except Exception as e:
    print(f"An error occurred: {e}")
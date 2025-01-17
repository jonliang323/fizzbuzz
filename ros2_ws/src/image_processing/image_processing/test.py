import numpy as np
import cv2
import os

try:
    # List all files in the folder
    folder_path = './pics'
    files = os.listdir(folder_path)

    # Loop through the files
    for i, file_name in enumerate(files):
        # Build the new file name
        old_file_path = os.path.join(folder_path, file_name)

        # Create the new file name
        new_file_name = f"{i}.jpg"
        new_file_path = os.path.join(folder_path, new_file_name)

        # Rename the file
        os.rename(old_file_path, new_file_path)

except Exception as e:
    print(f"An error occurred: {e}")
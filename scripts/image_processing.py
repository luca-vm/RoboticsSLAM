import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt


def dilate_map(occupancy_grid, kernel_size = 5):
    kernel = np.ones((kernel_size,kernel_size), np.uint8)
    height = occupancy_grid.info.height
    width = occupancy_grid.info.width
    nparray = np.array(occupancy_grid.data).reshape((height, width))
    
    cv_image = np.uint8((nparray == 100) * 255)
    dilated_image = cv2.dilate(cv_image, kernel, iterations=1)
    
    
    # Convert dilated image back to occupancy grid format
    dilated_grid = (dilated_image == 255).astype(np.int8) * 100
    dilated_grid = dilated_grid.flatten().tolist()
    
    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plt.imshow(cv_image, cmap='gray')
    plt.title('Original Occupancy Grid')

    plt.subplot(1, 2, 2)
    plt.imshow(dilated_image, cmap='gray')
    plt.title('Dilated Occupancy Grid')

    plt.show()
    
    occupancy_grid.data = dilated_grid
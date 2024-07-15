#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped


def plot_gmap_neighbourhood(gmap,occupancy_grid_in,randomx, randomy):
    width = occupancy_grid_in.info.width
    height = occupancy_grid_in.info.height
    data = np.array(occupancy_grid_in.data).reshape((height, width))

    plt.figure(figsize=(10, 10))
    plt.imshow(data, cmap='gray', origin='lower')
    
    
    map_origin_x = occupancy_grid_in.info.origin.position.x
    map_origin_y = occupancy_grid_in.info.origin.position.y
    resolution = occupancy_grid_in.info.resolution
    
    # plt.figure()
    # plt.imshow(data, cmap='gray', origin='lower')
    
    # point
    
    grid_x = int((randomx[len(gmap) - 1] - map_origin_x) / resolution)
    grid_y = int((randomy[len(gmap) - 1] - map_origin_y) / resolution)
    
    
    plt.plot(grid_x, grid_y, 'go')
    
    # neighbours
    
    nbx = [randomx[i] for i in gmap[-1]]
    nby = [randomy[i] for i in gmap[-1]] 

    for i in range(len(nbx)):
        grid_x = int((nbx[i] - map_origin_x) / resolution)
        grid_y = int((nby[i] - map_origin_y) / resolution)
        
        plt.plot(grid_x, grid_y, 'ro')
        
    plt.title("GMAP")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

def plot_neighbours(co_ords_x,co_ords_y,neighbors_x,neighbors_y,point_in_x,point_in_y,occupancy_grid_in):
    width = occupancy_grid_in.info.width
    height = occupancy_grid_in.info.height
    data = np.array(occupancy_grid_in.data).reshape((height, width))

    plt.figure(figsize=(10, 10))
    plt.imshow(data, cmap='gray', origin='lower')

    # Plot the robot's current position on the map if available
    map_origin_x = occupancy_grid_in.info.origin.position.x
    map_origin_y = occupancy_grid_in.info.origin.position.y
    resolution = occupancy_grid_in.info.resolution
    print('map is ',occupancy_grid_in.info)
    # Convert pose position to map grid coordinates
    
    # all points
    for i in range(len(co_ords_x)):            
        grid_x = int((co_ords_x[i] - map_origin_x) / resolution)
        grid_y = int((co_ords_y[i] - map_origin_y) / resolution)
        
        plt.plot(grid_x, grid_y, 'ro')
        
    # neighbors
    for i in range(len(neighbors_y)):            
        grid_x = int((neighbors_x[i] - map_origin_x) / resolution)
        grid_y = int((neighbors_y[i] - map_origin_y) / resolution)
        
        plt.plot(grid_x, grid_y, 'bo')
        
    # current point
    grid_x = int((point_in_x - map_origin_x) / resolution)
    grid_y = int((point_in_y - map_origin_y) / resolution)
    plt.plot(grid_x, grid_y, 'go')
    
    
    plt.title("grid")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

    
def plot_map_coords(co_ords_x,co_ords_y,occupancy_grid_in):
    # Convert occupancy grid data to 2D array
    width = occupancy_grid_in.info.width
    height = occupancy_grid_in.info.height
    data = np.zeros((height, width))

    plt.figure()
    plt.imshow(data, cmap='gray', origin='lower')

    # Plot the robot's current position on the map if available
    map_origin_x = occupancy_grid_in.info.origin.position.x
    map_origin_y = occupancy_grid_in.info.origin.position.y
    resolution = occupancy_grid_in.info.resolution
    print('map is ',occupancy_grid_in.info)
    # Convert pose position to map grid coordinates
    for i in range(len(co_ords_x)):            
        grid_x = int((co_ords_x[i] - map_origin_x) / resolution)
        grid_y = int((co_ords_y[i] - map_origin_y) / resolution)
        
        plt.plot(grid_x, grid_y, 'ro')

    plt.title("grid")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()
    
def show_occ_map(occupancy_grid_in):
    height = occupancy_grid_in.info.height
    width = occupancy_grid_in.info.width
    nparray = np.array(occupancy_grid_in.data).reshape((height, width))
    
    plt.figure()
    plt.imshow(nparray, cmap='gray', origin='lower')
    plt.title("grid")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()
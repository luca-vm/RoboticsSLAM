#!/usr/bin/env python

import sys
import math
# from scipy.spatial import KDTree
import numpy as np
# from debug import Debug
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")
import time
from kd_tree import KDTree
from debug import plot_map_coords,plot_neighbours
from image_processing import dilate_map

class Node:
    def __init__(self, x, y, cost, parent_index, heuristic):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.heuristic = heuristic

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost)


def prm(sx, sy, gx, gy, occupancy_grid, robot_radius):
    dilate_map(occupancy_grid, 3)
    # get the obstacle coordinates from the occupancy grid in map space
    ox, oy = get_obstacle_coords(occupancy_grid)
    
    obstacle_points = list(zip(ox, oy))
    NN = KDTree(obstacle_points, dim=2)

    print("Start: ({:.2f}, {:.2f}), Goal: ({:.2f}, {:.2f})".format(sx, sy, gx, gy))
    print("Number of obstacles: {:.2f}".format(len(ox)))

    randomx, randomy = random_points(sx, sy, gx, gy, robot_radius, ox, oy, NN, occupancy_grid)
    
    print("Random points generated:")
    for x, y in zip(randomx[:10], randomy[:10]):
        print("({:.2f}, {:.2f})".format(x,y))
    print("...")

    road_map = generate_map(randomx, randomy, robot_radius, NN, occupancy_grid)
    
    print("Road map generated with {:.2f} nodes".format(len(road_map)))
    
    px, py = a_star_planning(sx, sy, gx, gy, road_map, randomx, randomy)
    
    px = px[::-1]
    py = py[::-1]
    path = zip(px,py)
    if px and py:
        print("\nPath found with {:.2f} nodes!".format(len(px)))
        print("Node positions in the final path:")
        for x, y in path:
            print("({:.2f}, {:.2f})".format(x,y))
    else:
        print("\nNo path found.")


    plot_neighbours(randomx,randomy,px,py,gx,gy,occupancy_grid_in=occupancy_grid)
    
    
    
    return path


def is_collision(sx, sy, gx, gy, rr, NN, occupancy_grid):
    val = math.hypot(gx - sx, gy - sy)

    if val >= 30.0:
        return True

    n = int(val / rr)
    # print(n, "val of n")
    for i in range(n):
        # print(i, "val of i")     
        x, y = sx + rr * i * math.cos(math.atan2(gy - sy, gx - sx)), sy + rr * i * math.sin(math.atan2(gy - sy, gx - sx))
        # print(x,":",y, "val of x,y")
        # if not is_within_bounds(occupancy_grid, x, y):
        #     return True
        
        nearest = NN.get_nearest([x, y], return_dist_sq=True)
        if nearest:
            dist = math.sqrt(nearest[0])
            if dist <= rr or occupancy_grid.data[get_grid_index(occupancy_grid, x, y)] > 50:
                return True

    nearest = NN.get_nearest([gx, gy], return_dist_sq=True)
    if nearest:
        dist = math.sqrt(nearest[0])
        if dist <= rr or occupancy_grid.data[get_grid_index(occupancy_grid, gx, gy)] > 50:
            return True

    return False


def generate_map(randomx, randomy, rr, NN, occupancy_grid):
    gmap = {}
    n = len(randomx)
    sample_points = list(zip(randomx, randomy))
    sampleNN = KDTree(sample_points, dim=2)
    
    for (r, rx, ry) in zip(range(n), randomx, randomy):
        nearest = sampleNN.get_knn([rx, ry], k=10, return_dist_sq=True)
        neighbors = []
        # neighbours_x = []
        # neighbours_y = []
        for dist_sq, point in nearest[1:]:  # Skip the first one as it's the point itself
            nx, ny = point
            
            if not is_collision(rx, ry, nx, ny, rr, NN, occupancy_grid):
                neighbors.append((nx, ny))
                # neighbours_x.append(nx)
                # neighbours_y.append(ny)
            if len(neighbors) >= 10:
                break
        
        gmap[(rx, ry)] = neighbors
        # plot_neighbours(randomx,randomy,neighbours_x,neighbours_y,rx,ry,occupancy_grid_in=occupancy_grid)
    return gmap



def plot_open_closed(open_set, closed_set, road_map, randomx, randomy, sx, sy, gx, gy):
    plt.clf()  # Clear the current figure

    # Plot road map
    for i in range(len(road_map)):
        x1, y1 = randomx[i], randomy[i]
        plt.plot(x1, y1, 'bo')

    
    # Plot open set
    if(open_set):
        for node in open_set.values():
            if (node.parent_index !=-1):
                parentx = randomx[node.parent_index]
                parenty = randomy[node.parent_index]
                plt.plot([node.x,node.y],[parentx,parenty], 'go-')  # green lines
            else:
                plt.plot(node.x, node.y, 'bo')  # blue circles
    else:
        print("No open set")
    # Plot closed set
    if(closed_set):
        for node in closed_set.values():
            plt.plot(node.x, node.y, 'ro')  # red circles
    else:
        print("No closed set")

    # Plot start and goal points
    plt.plot(sx, sy, 'bs', label="Start")  # blue square
    plt.plot(gx, gy, 'rs', label="Goal")  # red square

    plt.legend()
    plt.pause(5)  # Pause for a bit to allow the plot to update
    
    
def a_star_planning(sx, sy, gx, gy, road_map, randomx, randomy):
    
    cost = 0.0
    start_node = Node(sx, sy, cost, -1, euclidean_distance(sx, sy, gx, gy))
    goal_node = Node(gx, gy, cost, -1, 0.0)

    open_set = {}
    closed_set = {}
    start_tuple = (sx, sy)
    goal_tuple = (gx, gy)
    open_set[start_tuple] = start_node
    
    # ---- check if goal and start have neighbours ------- 
    if len(road_map[start_tuple]) == 0 or len(road_map[goal_tuple]) == 0:
        print("Start or Goal have no neighbours!")
        return [], []
    
    while open_set:
        c_id = min(open_set, key=lambda o: open_set[o].cost + open_set[o].heuristic)
        current = open_set[c_id]

        if c_id == goal_tuple:
            print("Goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        del open_set[c_id]
        closed_set[c_id] = current

        for neighbor in road_map.get(c_id, []):
            dx = neighbor[0] - current.x
            dy = neighbor[1] - current.y
            d = math.hypot(dx, dy)
            node = Node(neighbor[0], neighbor[1], current.cost + d, c_id, euclidean_distance(neighbor[0], neighbor[1], gx, gy))

            if neighbor in closed_set:
                continue

            if neighbor in open_set:
                if open_set[neighbor].cost > node.cost:
                    open_set[neighbor].cost = node.cost
                    open_set[neighbor].parent_index = c_id
            else:
                open_set[neighbor] = node
    else:
        # If open_set is empty and goal is not reached
        print("No path found!")
        return [], []
    
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index
        
    return rx, ry


def get_obstacle_coords(occupancy_grid):
    ox, oy = [], []
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    resolution = occupancy_grid.info.resolution
    map_origin_x = occupancy_grid.info.origin.position.x
    map_origin_y = occupancy_grid.info.origin.position.y
    # furked, need to add origin to conversion
    for i in range(height):
        for j in range(width):
            idx = i * width + j
            if occupancy_grid.data[idx] > 50:  # Consider cells with occupancy probability > 50% as obstacles
                ox.append(j * resolution + map_origin_x)
                oy.append(i * resolution  + map_origin_y)
    # plot_map_coords(ox, oy,occupancy_grid_in=occupancy_grid)

    return ox, oy

def euclidean_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def get_grid_index(occupancy_grid, x, y):
    map_origin_x = occupancy_grid.info.origin.position.x
    map_origin_y = occupancy_grid.info.origin.position.y
    resolution = occupancy_grid.info.resolution

    grid_x = int((x - map_origin_x) / resolution)
    grid_y = int((y - map_origin_y) / resolution)

    return grid_y * occupancy_grid.info.width + grid_x

# def is_within_bounds(occupancy_grid, x, y):
#     map_origin_x = occupancy_grid.info.origin.position.x
#     map_origin_y = occupancy_grid.info.origin.position.y
#     resolution = occupancy_grid.info.resolution
#     width = occupancy_grid.info.width
#     height = occupancy_grid.info.height

#     grid_x = int((x - map_origin_x) / resolution)
#     grid_y = int((y - map_origin_y) / resolution)

#     return 0 <= grid_x < width and 0 <= grid_y < height

def random_points(sx, sy, gx, gy, rr, ox, oy, NN, occupancy_grid):
    randomx = []
    randomy = []

    # Use the occupancy grid to determine map bounds
    resolution = occupancy_grid.info.resolution
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y

    # finding the bounds of the map in with regards to the size of the occupancy gridz
    max_x = 8
    max_y = 15
    
    min_x = -15
    min_y = -7

    print("Map bounds: ({:.2f}, {:.2f}) to ({:.2f}, {:.2f})".format(min_x,min_y,max_x,max_y))

    while len(randomx) <= 700:
        x = np.random.uniform(min_x, max_x)
        y = np.random.uniform(min_y, max_y)
        
    
        grid_index = get_grid_index(occupancy_grid, x, y)
        if occupancy_grid.data[grid_index] <= 50:  # Check if the cell is free  
            # x and y are in map space, therefore randomx and randomy are in map space
            randomx.append(x)
            randomy.append(y)
            
    # Include start and goal to the randomly sampled points
    
    
    # check start point
    grid_index = get_grid_index(occupancy_grid, sx, sy)
    if occupancy_grid.data[grid_index] <= 50:  # Check if the cell is free  
        # x and y are in map space, therefore randomx and randomy are in map space
        randomx.append(sx)
        randomy.append(sy)
    else:
        print("Start point is an obstacle!")
    # check goal point
    grid_index = get_grid_index(occupancy_grid, gx, gy)
    if occupancy_grid.data[grid_index] <= 50:  # Check if the cell is free  
        # x and y are in map space, therefore randomx and randomy are in map space
        randomx.append(gx)
        randomy.append(gy)
    else:
        print("Goal point is an obstacle!")
    
    # plot_map_coords(randomx, randomy, occupancy_grid_in=occupancy_grid)
    print("Generated {:.2f} valid random points".format(len(randomx)))
    return randomx, randomy
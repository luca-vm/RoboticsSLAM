#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from pid import PID
from math import pow, sqrt
import math
from prm import prm

class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_pose)

        self.pose = None
        self.occupancy_grid = None
        self.rate = rospy.Rate(10)
        self.pid = PID()  # Initialize the PID instance

        # Get the occupancy grid map from the map server
        self.occupancy_grid = self.get_occupancy_grid()

    def update_pose(self, pose_msg):
        self.pose = pose_msg

    def get_occupancy_grid(self):
        rospy.wait_for_service('static_map')
        try:
            get_map = rospy.ServiceProxy('static_map', GetMap)
            response = get_map()
            return response.map
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return None

    # In TurtleBot class move2goal method
    def move2goal(self, goal_pose):
        vel_msg = Twist()
        while not rospy.is_shutdown():
            if self.pose is None or self.occupancy_grid is None:
                continue

            x, y, yaw = self.pid.get_state(self.pose)
            
            # Calculate distance and angle to goal
            distance = self.euclidean_distance(x, y, goal_pose.x, goal_pose.y)
            angle_to_goal = math.atan2(goal_pose.y - y, goal_pose.x - x)
            
            # Adjust yaw to be between -pi and pi
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            
            # Calculate angular difference
            angular_difference = angle_to_goal - yaw
            angular_difference = math.atan2(math.sin(angular_difference), math.cos(angular_difference))

            # PID control
            linear_speed = self.pid.compute_pid(distance)
            angular_speed = self.pid.compute_pid_angular(yaw, angle_to_goal)

            # print(angular_difference)
            # Ensure rotation is in the correct direction
            if abs(angular_difference) > 0.2:  # Threshold for starting rotation
                vel_msg.linear.x = 0
                vel_msg.angular.z = min(0.5, max(-0.5, angular_speed))  # Limit angular speed
            else:
                vel_msg.angular.z = 0
                vel_msg.linear.x = min(0.2, max(-0.2, linear_speed))  # Limit linear speed

            # Publish velocity
            self.velocity_publisher.publish(vel_msg)
        
            # Check if we have reached the goal
            if distance < 0.1:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                print("Reached intermediate goal: ({:.2f}, {:.2f})".format(goal_pose.x, goal_pose.y))
                break

            self.rate.sleep()


    def euclidean_distance(self, x1, y1, x2, y2):
        return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))
            
    def follow_path(self, path):
    # Reverse the path to go from start to goal
        # path[0].reverse()
        # path[1].reverse()
        # path[0][::-1]
        # path[1][::-1]
        print("first point", path[0])
        for step in path:
            print("step",step)
            print("len(step)",len(step))
            goal_pose = Point()
            goal_pose.x = step[0]
            goal_pose.y = step[1]

            print("Moving to point({:.2f}, {:.2f})".format(goal_pose.x,goal_pose.y))
            self.move2goal(goal_pose)
            rospy.sleep(1)  # Short pause between goals
            
if __name__ == '__main__':
    
    try:
        x = TurtleBot()

        while x.pose is None or x.occupancy_grid is None:
            rospy.sleep(0.1)

        gx = float(input('Input the goal x value: '))
        gy = float(input('Input the goal y value: '))

        sx, sy, _ = x.pid.get_state(x.pose)

        print("\nStarting PID from ({:.2f}, {:.2f}) to ({:.2f}, {:.2f})".format(sx, sy, gx, gy))
        
        
        path = ([],[])
        
        while(not path[0] and not path[1]):
            path = prm(sx, sy, gx, gy, x.occupancy_grid, 0.1)
            print()
            # path =  [(gx, gy)] 
            if path[0] and path[1]:
                print("\nExecuting path...")
                x.follow_path(path)
            else:
                #add a shift to the start and goal
                print("No path found. Unable to reach the goal.")

    except rospy.ROSInterruptException:
        pass 
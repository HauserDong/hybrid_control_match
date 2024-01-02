#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
import tf

def costmap_callback(data):
    
    # check a point (x,y) cost in costmap
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    origin = data.info.origin

    cost_data = data.data   # 1-D array, in row-major order
    
    map_x = int((x_check-origin.position.x)/resolution)
    map_y = int((y_check-origin.position.y)/resolution)

    # index = map_y * width + map_x
    # cost = cost_data[index]
    # print(x_check,",",y_check,"cost:",cost)

    global msg_received, near_obstacle
    msg_received = True

    # check its neighbouring point, avoid obstructing by algorithm
    check_radius = 0.35     # in meter
    check_window_width = int((2*check_radius)/resolution)
    check_window_origin_x = int(map_x - check_window_width/2)
    check_window_origin_y = int(map_y - check_window_width/2)

    for i in range(check_window_width):
        for j in range(check_window_width):
            idx = (check_window_origin_y + i) * width + (check_window_origin_x + j)
            if cost_data[idx] != 0:
                near_obstacle = True
                break    

def is_goal_feasible(input_goal):
    # Check goal feasibility
    # Input: x, y of goal (not support orientation now)
    # Output: Bool

    global x_check, y_check
    x_check = input_goal[0]
    y_check = input_goal[1]

    # global msg_received, near_obstacle
    # msg_received = False        # received message or not
    # near_obstacle = False       # is the point near obstacle (avoid safe but near obstacle points)

    rospy.wait_for_service('move_base/make_plan')   # move_base/make_plan gets a plan from the start to goal

    try:
        make_plan = rospy.ServiceProxy('move_base/make_plan',GetPlan)
        
        # Here we use a predefined start point, same with simulation setting
        start = PoseStamped()
        start.header.stamp = rospy.Time.now()
        start.header.frame_id = 'map'
        start.pose.position.x = 2.0
        start.pose.position.y = -2.0
        start.pose.position.z = 0.0
        start.pose.orientation.x = 0.0
        start.pose.orientation.y = 0.0
        start.pose.orientation.z = 0.0
        start.pose.orientation.w = 1.0
        

        # Define the goal
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose = Pose(Point(input_goal[0], input_goal[1], 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))

        # Define tolerance
        tolerance = 0.0

        response = make_plan(start,goal,tolerance)  # call the service

        if len(response.plan.poses)>0:
            
            # rospy.Subscriber("/map",OccupancyGrid,costmap_callback)
            # rate = rospy.Rate(10)

            # while not rospy.is_shutdown():
            #     if not msg_received:
            #         rate.sleep()
            #     else:
            #         break
            
            return True     # Goal is feasible
        else:
            return False    # Goal is not feasible

    
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

def Get_goal():

    rospy.loginfo("Input the x and y coordinates of goals and their corresponding priority levels.")
    rospy.loginfo("Hint: x ranges from [-3,3], y ranges from [-2.5,3.5], priority levels are positive integers")

    coordinates = []  
    while True:
        try:
            x, y, p = map(float, input("Please enter the x and y coordinates and priority level, separated by a space: ").split())
            coordinate = (x, y, int(p))
    
            print(f"Check feasibility for {coordinate} ...")
            flag = is_goal_feasible(coordinate)

            if flag:
                if coordinate[2] <= 0:
                    rospy.logwarn("Please choose a positive integer for priority level")
                    continue
                else:
                    coordinates.append(coordinate) 
                    print(f"Coordinate {coordinate} saved successfully.")
            else:
                rospy.logwarn(f"Sorry, {coordinate} is not feasible. Please try another goal.")
                continue
            
            continue_input = input("If you have finished your input, please type 'q'. If you want to continue, type anything else: ")
            if continue_input.lower() == 'q':
                break
        except ValueError:
            rospy.logwarn("Invalid input. Please enter numerical values for coordinates and its priority level separated by a space.")

    rospy.loginfo(f"Coordinates {coordinates} were recorded. Total number of coordinates: {len(coordinates)}")

    coordinates_dict = dict()
    for i in range(len(coordinates)):
        coordinates_dict[str(i+1)] = [Pose(Point(coordinates[i][0], coordinates[i][1], 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000)),coordinates[i][2]]
    
    return coordinates_dict

if __name__ == "__main__":
    rospy.init_node("get_goal")
    Get_goal()
    rospy.spin()
#!/usr/bin/env python3

# --------Include modules---------------
from copy import copy
import rospy
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
import tf
from mbot_explore.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot, informationGain, discount, goalVisual
from numpy.linalg import norm
import math

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
globalmaps = []
flag = 0.
rescue_goal = []


def callBack(data):
    global frontiers
    frontiers = []
    for point in data.points:
        frontiers.append(array([point.x, point.y]))


def mapCallBack(data):
    global mapData
    mapData = data


def rescue_callback(msg):
    global flag, rescue_goal
    if flag < 0.5:
        # rospy.loginfo("got a rescue goal")
        rescue_goal.clear()
        if math.isnan(msg.point.x):
            # flag = 0
            return
        rescue_goal.append(msg.point.x)
        rescue_goal.append(msg.point.y)
        flag = 1.
        rospy.loginfo("Got a rescue goal: " + str(rescue_goal))


def flag_callback(msg):
    global flag
    flag = msg.data
    rospy.loginfo("Continue exploring--------------------------------------")


# Node----------------------------------------------

def node():
    global frontiers, mapData, globalmaps, flag, rescue_goal
    rospy.init_node('assigner', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    info_radius = rospy.get_param('~info_radius',
                                  1.0)  # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_multiplier = rospy.get_param('~info_multiplier', 3.0)
    hysteresis_radius = rospy.get_param('~hysteresis_radius', 3.0)  # at least as much as the laser scanner range
    hysteresis_gain = rospy.get_param('~hysteresis_gain',
                                      2.0)  # bigger than 1 (biase robot to continue exploring current region
    frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
    delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.5)
    rateHz = rospy.get_param('~rate', 100)
    robot_namelist = rospy.get_param('~robot_namelist', "")

    rate = rospy.Rate(rateHz)
    # -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)
    rospy.Subscriber('object_detect_pose', PointStamped, rescue_callback)
    rospy.Subscriber('flag', Int32, flag_callback)
    # ---------------------------------------------------------------------------------------------------------------
    # Publish the goal for visual
    goal_publisher = rospy.Publisher('goal_visual', Marker, queue_size=10)
    position_publisher = rospy.Publisher('robot_position', PointStamped, queue_size=10)
    reach_publisher = rospy.Publisher('reach', Int32, queue_size=10)
    # ---------------------------------------------------------------------------------------------------------------

    # perform name splitting for the robot
    robot_namelist = robot_namelist.split(',')
    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass
    centroids = copy(frontiers)
    # wait if map is not received yet
    while (len(mapData.data) < 1):
        pass

    robots = []
    for i in range(0, len(robot_namelist)):
        robots.append(robot(name=robot_namelist[i]))

    for i in range(0, len(robot_namelist)):
        robots[i].sendGoal(robots[i].getPosition())
    # -------------------------------------------------------------------------
    # ---------------------     Main   Loop     -------------------------------
    # -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        centroids = copy(frontiers)
        last_rescue_goal = []
        # -------------------------------------------------------------------------
        # Get information gain for each frontier point
        infoGain = []
        for ip in range(0, len(centroids)):
            infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))
        # -------------------------------------------------------------------------
        # get number of available/busy robots
        na = []  # available robots
        nb = []  # busy robots
        for i in range(0, len(robot_namelist)):
            if (robots[i].getState() == 1):
                nb.append(i)
            else:
                na.append(i)
        # rospy.loginfo("available robots: " + str(na))
        # -------------------------------------------------------------------------
        # get dicount and update informationGain
        for i in nb + na:
            infoGain = discount(mapData, robots[i].assigned_point, centroids, infoGain, info_radius)
        # -------------------------------------------------------------------------
        revenue_record = []
        centroid_record = []
        id_record = []

        for ir in na:
            for ip in range(0, len(centroids)):
                cost = norm(robots[ir].getPosition() - centroids[ip])
                threshold = 1
                information_gain = infoGain[ip]
                if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
                    information_gain *= hysteresis_gain
                revenue = information_gain * info_multiplier - cost
                revenue_record.append(revenue)
                centroid_record.append(centroids[ip])
                id_record.append(ir)

        if len(na) < 1:
            revenue_record = []
            centroid_record = []
            id_record = []
            for ir in nb:
                for ip in range(0, len(centroids)):
                    cost = norm(robots[ir].getPosition() - centroids[ip])
                    threshold = 1
                    information_gain = infoGain[ip]
                    if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
                        information_gain *= hysteresis_gain

                    if ((norm(centroids[ip] - robots[ir].assigned_point)) < hysteresis_radius):
                        information_gain = informationGain(mapData, [centroids[ip][0], centroids[ip][1]],
                                                           info_radius) * hysteresis_gain

                    revenue = information_gain * info_multiplier - cost
                    revenue_record.append(revenue)
                    centroid_record.append(centroids[ip])
                    id_record.append(ir)

        # rospy.loginfo("revenue record: " + str(revenue_record))
        # rospy.loginfo("centroid record: " + str(centroid_record))
        # rospy.loginfo("robot IDs record: "+str(id_record))

        # -------------------------------------------------------------------------
        if (len(id_record) > 0):
            winner_id = revenue_record.index(max(revenue_record))

            # Cancel recent goal
            if flag > 0.5:
                robots[id_record[winner_id]].cancelGoal()
                robot_state = robots[id_record[winner_id]].getState()
                rospy.loginfo("Robot state is: " + str(robot_state))
                # print(rescue_goal)

                goal = goalVisual(rescue_goal)
                goal.header.stamp = rospy.Time.now()
                goal.lifetime = rospy.Duration()
                goal_publisher.publish(goal)

                robots[id_record[winner_id]].sendGoal(rescue_goal)
                rospy.loginfo("Robot is going to: " + str(rescue_goal))

            # On the way to the rescue!
            while flag > 0.5:
                # if rescue_goal != last_rescue_goal:
                if True:
                    goal = goalVisual(rescue_goal)
                    goal.header.stamp = rospy.Time.now()
                    goal.lifetime = rospy.Duration()
                    goal_publisher.publish(goal)
                    robots[id_record[winner_id]].sendGoal(rescue_goal)
                    rospy.loginfo("Robot is going to: " + str(rescue_goal))
                    last_rescue_goal = rescue_goal

                rospy.loginfo("On the way to the rescue: " + str(rescue_goal))
                rospy.Rate(0.5).sleep()
                position_publisher.publish(robots[id_record[winner_id]].pub_position())

                if robots[id_record[winner_id]].rescue_goal_distance(rescue_goal) < 0.3:
                    reach = 1
                    rospy.loginfo("Reached the rescue goal!!!!!")
                    reach_publisher.publish(reach)
                pass

            if flag < 0.5:
                goal = goalVisual(centroid_record[winner_id])
                goal.header.stamp = rospy.Time.now()
                goal.lifetime = rospy.Duration()
                goal_publisher.publish(goal)

                robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
                # print(centroid_record[winner_id])
                rospy.loginfo(
                    robot_namelist[id_record[winner_id]] + "  assigned to  " + str(centroid_record[winner_id]))
                rospy.sleep(delay_after_assignement)

        # -------------------------------------------------------------------------
        rate.sleep()


# -------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass

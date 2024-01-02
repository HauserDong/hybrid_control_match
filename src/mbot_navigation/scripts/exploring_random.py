#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  
from target_input import *

def cal_path_len(s,g):
    rospy.wait_for_service('move_base/make_plan')

    make_plan = rospy.ServiceProxy('move_base/make_plan',GetPlan)

    start = PoseStamped()
    start.header.stamp = rospy.Time.now()
    start.header.frame_id = 'map'
    start.pose = s

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'
    goal.pose = g

    tolerance = 0.0

    response = make_plan(start,goal,tolerance)  # call the service

    return len(response.plan.poses)

def resorted(sort_list):
    # Resort the Target list, especially for those who have the same priority level
    # input: target list after using sorted function
    # output: a new target list rearanging those targets with same priority level
    
    last_point = Pose(Point(2.000, -2.000, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))
    total_num = len(sort_list)
    ptr = 0 
    

    while True:
        if ptr == total_num:
            break

        same_prior_list = []
        prior = sort_list[ptr][1][1]

        while True:
            if ptr == total_num:
                break

            if sort_list[ptr][1][1] == prior:
                goal = sort_list[ptr][1][0]
                path_len = cal_path_len(last_point,goal)

                same_prior_list.append(sort_list[ptr])
                same_prior_list[-1][1].append(path_len)
            else:
                break

            ptr += 1
        
        sort_same_prior_list = sorted(same_prior_list,key=lambda x:x[1][2]) # increasing order

        ptr_start = ptr - len(same_prior_list)
        ptr_end = ptr
        j = 0
        for i in range(ptr_start,ptr_end):
            sort_list[i] = sort_same_prior_list[j]
            j += 1
        
        last_point = sort_same_prior_list[j-1][1][0]

class NavTest():  
    def __init__(self):  
        rospy.init_node('exploring_random', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 2)  

        # 是否仿真？  
        self.fake_test = rospy.get_param("~fake_test", True)  

        # 到达目标的状态  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
 
        # 获取用户输入的目标位置
        locations_dict = Get_goal()
        
        # 按照优先级从高到底，对目标进行排序
        sorted_locations = sorted(locations_dict.items(), key=lambda x:x[1][1],reverse=True)

        resorted(sorted_locations)

        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  

        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
  
        # 保存机器人的在rviz中的初始位置  
        initial_pose = PoseWithCovarianceStamped()  

        # 保存成功率、运行时间、和距离的变量  
        n_locations = len(sorted_locations)  
        n_goals = 0  
        n_successes = 0  
        i = 0
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = ""    
 
        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  

        rospy.loginfo("Starting navigation test")

        # 开始主循环，按照优先级导航 
        while not rospy.is_shutdown():  
            if i == n_locations:
                break

            # 在当前的排序中获取下一个目标点 
            location = sorted_locations[i][0]  

            # 计数器加1  
            i += 1  
            n_goals += 1  

            # 设定下一个目标点  
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = locations_dict[location][0] 
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  

            # 让用户知道下一个位置  
            rospy.loginfo("Going to: " + str(location)+". Its coordinate is ("+str(locations_dict[location][0].position.x)+","+str(locations_dict[location][0].position.y)+"). Its priority is "+str(locations_dict[location][1]))  

            # 向下一个位置进发  
            self.move_base.send_goal(self.goal)  

            # 五分钟时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 

            # 查看是否成功到达  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    n_successes += 1  
                    # distance_traveled += distance  
                    rospy.loginfo("State:" + str(state))  
                else:  
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  

            # 运行所用时间  
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0  

            # 输出本次导航的所有信息  
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +   
                          str(n_goals) + " = " +   
                          str(100 * n_successes/n_goals) + "%")  

            rospy.loginfo("Running time: " + str(trunc(running_time, 1)))  

            rospy.sleep(self.rest_time)  

    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  

def trunc(f, n):  
    slen = len('%.*f' % (n, f))  

    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try:  
        NavTest()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")

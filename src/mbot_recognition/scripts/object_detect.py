#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
from scipy.stats import norm
from math import *
from geometry_msgs.msg import Pose,PointStamped
import tf2_ros
import tf2_geometry_msgs

# HSV相关阈值设定
HUE_LOW   = 0
HUE_HIGH  = 10
SATURATION_LOW  = 120
SATURATION_HIGH = 255
VALUE_LOW    = 70
VALUE_HIGH   = 255

# 识别目标相关参数设定
min_area = 1000     # 最小检测面积
max_area = 16000   # 最大检测面积
view_center = 320   # 视野中心像素x坐标
view_bias = 150     # 容许观察视野的半径（单位：像素）
image_topic_hz = 12 # 图像数据发布频率（单位：hz），通过rostopic hz获取
get_data_time = 0.5 # 获取数据时长（单位：s）
THRESHOLD = 0.05    # 对点集中点分类时的距离标度

def distance(a,b):
    p_a = np.array([a[1].point.x,a[1].point.y])
    p_b = np.array([b[1].point.x,b[1].point.y])
    return np.linalg.norm(p_a-p_b)

def group_points(point_list,threshold):
    # 为已有的点分组，并返回最聚集的点族

    groups = []

    for point in point_list:
        for group in groups:
            if any(distance(point,point_in_group)<=threshold for point_in_group in group):
                group.append(point)
                break
        else:
            groups.append([point])
    
    group_len = [len(group) for group in groups]
    max_idx = group_len.index(max(group_len))

    return groups[max_idx]



class image_converter:
    # 筛选最终发布目标点相关参数
    num = 0     # 记录收到多少个视觉msg
    min_dist_list = []  # 记录每次接受msg后最近目标坐标

    def __init__(self):    
    # 创建cv_bridge，声明图像的发布者和订阅者
      self.bridge=CvBridge()	#ROS图像和OpenCV图像信息的转换
      self.image_sub=rospy.Subscriber("/limo/color/image_raw", Image, self.visual_callback)	#订阅仿真中Camera的话题
      self.depth_sub=rospy.Subscriber("/limo/depth/image_raw",Image,self.depth_callback)    # 获取仿真中深度信息
      self.camera_info_sub = rospy.Subscriber("/limo/color/camera_info",CameraInfo,self.camera_info_callback)   # 获取相机内参
      self.image_pub=rospy.Publisher("object_detect_image", Image, queue_size=1)	#发布识别结果
      self.target_pub=rospy.Publisher("/object_detect_pose", PointStamped, queue_size=1)	#发布target的Pose信息
      self.tf_buffer = tf2_ros.Buffer()
      self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_pose(self):
        trans = self.tf_buffer.lookup_transform('odom','base_footprint',rospy.Time(0))
        return [trans.transform.translation.x,trans.transform.translation.y]

    def visual_callback(self,data):
        
        self.num += 1

        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            image_input = self.bridge.imgmsg_to_cv2(data, "bgr8")	#将ROS中拿到的数据转换成OpenCV能够使用的数据
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.cvtColor(image_input,cv2.COLOR_BGR2HSV)  #将获得的bgr图转化为hsv图，这样更利于我们在真实环境中识别物体
        # print("Size of image:", cv_image.shape)   #(480,640,3)

        # define the list of boundaries in BGR 
        boundaries = [([HUE_LOW, SATURATION_LOW, VALUE_LOW], [HUE_HIGH,SATURATION_HIGH,VALUE_HIGH])]	#识别颜色的范围值BGR

        # loop over the boundaries
        for (lower, upper) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

        # find the colors within the specified boundaries and apply the mask
        mask = cv2.inRange(cv_image, lower, upper)      # mask中颜色区域内的像素值为255，其他区域像素值为0，lower表示每个通道的下界

        cnts,_ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        dist_list = []
        center_list = []

        # 获取机器人自身坐标
        trans = self.get_pose()
        robot_pos = np.array(trans)
        # rospy.loginfo("Robot pos:"+str(trans))

        # loop over the contours
        for c in cnts:  # 每个c是一个轮廓的轮廓点组成的数组
            # compute the center of the contour
            M = cv2.moments(c)
            # print("M[m00]:",M["m00"])

            if int(M["m00"]) <= min_area or int(M["m00"]) >= max_area:	# M["m00"]是面积，1000是一个比较适合的下界，需要观察的面积足够大
                continue
            # print("Area: ", int(M["m00"]))
            
            cX = int(M["m10"] / M["m00"])   # M["m10"]是一阶矩，用来计算质心位置
            cY = int(M["m01"] / M["m00"])   # M["m10"]是一阶矩，用来计算质心位置

            if cX not in range(view_center - view_bias, view_center + view_bias):        # 观察到的物体尽量在视野中央，避免在边缘处产生较大估计误差
                continue
            
            # 在图像上显示识别出的物体轮廓和质心
            cv2.drawContours(image_input, [c], -1, (255, 68, 0), 2)
            cv2.circle(image_input, (cX, cY), 1, (0, 255, 230), -1)

            # 获取对应质心的深度信息
            depth = self.depth_image[cY,cX]
            # rospy.loginfo("Robot detecting: get a target with depth "+str(depth)+"m")

            # 计算物体在相机坐标系中的位置,像素坐标系-->相机坐标系
            point_camera = PointStamped()
            point_camera.header.frame_id = data.header.frame_id
            point_camera.point.x = (cX - self.camera_info.K[2]) * depth / self.camera_info.K[0]
            point_camera.point.y = (cY - self.camera_info.K[5]) * depth / self.camera_info.K[4]
            point_camera.point.z = depth

            # 使用tf2将点从相机坐标系转换到世界坐标系
            point_world = self.tf_buffer.transform(point_camera,"odom")
            # rospy.loginfo("Robot detecting: target position\n"+str(point_world))
            # self.target_pub.publish(point_world)

            target_pos = np.array([point_world.point.x,point_world.point.y])
            distance = np.linalg.norm(target_pos-robot_pos)
            dist_list.append(distance)
            center_list.append([cX,point_world])     # cX 用来加权，target_pos为坐标信息
        
        if len(dist_list) > 0:
            # 寻找距离最近的目标
            min_idx = dist_list.index(min(dist_list))
            self.min_dist_list.append(center_list[min_idx])

            # 发布目标位置
            self.publish_target()
        
        # 显示Opencv格式的图像
        cv2.imshow("Image window", image_input)
        # cv2.imshow("Image window", mask)
        cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def depth_callback(self,data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data,"32FC1")  # "32FC1"表示32位浮点数单通道，常用于表示深度图像

    def camera_info_callback(self,data):
        self.camera_info = data

    def publish_target(self):

        if self.num >= int(get_data_time*image_topic_hz):   # 积累短时间内一定数量的数据

            group = group_points(self.min_dist_list,THRESHOLD)
            group_len = len(group)

            # 按照cX的位置按照正态分布来赋予权重
            cX_data = np.array([point[0] for point in group])
            mean = view_center
            std = view_bias
            weights = norm.pdf(cX_data,mean,std)
            weights = weights/np.sum(weights)

            point_world = PointStamped()
            point_world.header.frame_id = "odom"
            point_world.point.x = sum(weights[i]*group[i][1].point.x for i in range(group_len))  # 计算加权
            point_world.point.y = sum(weights[i]*group[i][1].point.y for i in range(group_len))
            point_world.point.z = sum(weights[i]*group[i][1].point.z for i in range(group_len))
        
            self.target_pub.publish(point_world)

            rospy.loginfo("Robot detecting: target position\n"+str(point_world))

            self.num = 0
            self.min_dist_list = []



if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("object_detect")
        rospy.loginfo("Starting detect object")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down object_detect node.")
        cv2.destroyAllWindows()
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from math import sin, cos, radians, pi



rospy.init_node('front')
#the name of the topic
odom_pub = rospy.Publisher('/Pepper_odom', Odometry, queue_size=10)

rospy.wait_for_service ('gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

odom = Odometry()
header = Header()
header. frame_id = '/odom'

model = GetModelStateRequest()
#Here you put the name of your robot
model.model_name = 'pepper_MP'

result = get_model_srv(model)

r = rospy.Rate(1)

inf = float('inf')


while not rospy.is_shutdown():


    result = get_model_srv(model)
    odom.pose.pose.position.x = result.pose.position.x
    odom.pose.pose.position.y = result.pose.position.y
    odom.pose.pose.orientation.x = result.pose.orientation.x
    odom.pose.pose.orientation.y = result.pose.orientation.y
    robot_x = odom.pose.pose.position.x
    robot_y = odom.pose.pose.position.y
    robotO_x = odom.pose.pose.orientation.x
    robotO_y = odom.pose.pose.orientation.y
    
    def callback(msg):
        for distance in msg.ranges:
            if(distance != inf):
                ray_angle = msg.angle_min + (distance * msg.angle_increment)
	        print 'angulo:',ray_angle
                print 'distancia:', distance
                theta_rad = ray_angle*(pi/180)
                print '\n\tPepper -> x:', robot_x
                print '\tPepper -> y:', robot_y
                print '\tObstacle -> x:', robot_x + distance*cos(theta_rad)
                print '\tObstacle -> y:', robot_y + distance*sin(theta_rad)
                print('\n-------')
		
                
    
    rospy.init_node('front')
    sub = rospy.Subscriber('/pepper/scan_front', LaserScan, callback)
    r.sleep()
    rospy.spin()
    

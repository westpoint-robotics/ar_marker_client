#!/usr/bin/python

import roslib
import rospy
from sensor_msgs.msg import CameraInfo
import yaml

cam_info_publisher = None
cam_info_msg = None

def cam_info_callback(msg):
	cam_info_msg.header = msg.header
	cam_info_publisher.publish(cam_info_msg)

if __name__ == '__main__':
    
    rospy.init_node('cam_info_publisher')
    
    inputYaml = open('/home/thaus/.ros/camera_info/00b09d0100cd42b4.yaml', 'r')
    yamlData = yaml.load(inputYaml)
    inputYaml.close()
    
    cam_info_msg = CameraInfo()
    cam_info_msg.height = yamlData['image_height']
    cam_info_msg.width = yamlData['image_width']
    cam_info_msg.distortion_model = yamlData['distortion_model']
    cam_info_msg.D = yamlData['distortion_coefficients']['data']
    cam_info_msg.R = yamlData['rectification_matrix']['data']
    cam_info_msg.P = yamlData['projection_matrix']['data']
    cam_info_msg.K = yamlData['camera_matrix']['data']  
    
    cam_info_sub = rospy.Subscriber('/camera/camera_info', CameraInfo, cam_info_callback)
    cam_info_publisher = rospy.Publisher('/chameleon/camera_info', CameraInfo)
    
    while not rospy.is_shutdown():
		rospy.spin()


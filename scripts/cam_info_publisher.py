#!/usr/bin/python

import roslib
import rospy
from sensor_msgs.msg import CameraInfo, Image
import yaml
import rospkg

cam_info_publisher = None
cam_info_msg = None

def image_sub(msg):
    global cam_info_publisher
    global cam_info_msg
    cam_info_msg.header = msg.header
    cam_info_publisher.publish(cam_info_msg)
    

if __name__ == '__main__':
    
    rospy.init_node('cam_info_publisher')

    rospack = rospkg.RosPack()
    
    inputYaml = open(rospack.get_path('ar_marker_client') + '/cfg/euroc3_cam3.yaml', 'r')
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
    
    cam_info_publisher = rospy.Publisher('/euroc3/cam3/info', CameraInfo, queue_size=5)
    img_sub = rospy.Subscriber('/euroc3/cam3/image_raw', Image, image_sub)
    
    while not rospy.is_shutdown():
        rospy.sleep(0.01)
        #cam_info_msg.header.stamp = rospy.Time.now()
        #cam_info_msg.header.frame_id  = 'cam0'
    	#cam_info_publisher.publish(cam_info_msg)


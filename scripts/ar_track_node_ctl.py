#!/usr/bin/python

import roslib
import rospy
import roslaunch
from std_msgs.msg import Int16
import time
import signal

cam_info_publisher = None
cam_info_msg = None


class NodeCtl:

    def __init__(self):

        rospy.init_node('ar_track_node_ctl')

        self.rate = rospy.Rate(10)

        rospy.Subscriber('pack_track_state', Int16, self.mission_state_cb)
        self.detectionFlagPub = rospy.Publisher('detection_flag', Int16, 
            queue_size=1)

        signal.signal(signal.SIGINT, self.exit_cb)
        signal.signal(signal.SIGTERM, self.exit_cb)

        self.exit = 0
        self.ar_track_state = 0
        self.ar_track_state_old = 0

        self.package_ar = 'ar_track_alvar'
        self.exe_ar = 'individualMarkersNoKinect'

        self.package_mt = 'ar_marker_client'
        self.exe_mt = 'marker_tracker_node'

        self.package_ci = 'ar_marker_client'
        self.exe_ci = 'cam_info_publisher.py'

        self.marker_size = rospy.get_param('~marker_size')
        self.max_new_marker_error = rospy.get_param('~max_new_marker_error')
        self.max_track_error = rospy.get_param('~max_track_error')
        self.cam_image_topic = rospy.get_param('~cam_image_topic')
        self.cam_info_topic = rospy.get_param('~cam_info_topic')
        self.output_frame = rospy.get_param('~output_frame')
        self.package_id = rospy.get_param('~package_id')
        self.marker_offset_x = rospy.get_param('~marker_offset_x')
        self.marker_offset_y = rospy.get_param('~marker_offset_y')
        self.marker_offset_z = rospy.get_param('~marker_offset_z')
        self.cam2imuTf = rospy.get_param('~cam2imuTf')


        self.node_ar_track = roslaunch.core.Node(package=self.package_ar, node_type=self.exe_ar, name='ar_track_alvar',
                                                 namespace=rospy.get_namespace(),
                                                 args='{0} {1} {2} {3} {4} {5}'.format(self.marker_size, self.max_new_marker_error,
                                                                          self.max_track_error, self.cam_image_topic,
                                                                          self.cam_info_topic, self.output_frame))
        self.launch_ar_track = roslaunch.scriptapi.ROSLaunch()
        self.launch_ar_track.start()

        self.node_mt = roslaunch.core.Node(package=self.package_mt, node_type=self.exe_mt, name='marker_tracker',
                                           namespace=rospy.get_namespace(), output="screen", args='_marker_id_target:="{0}" _marker_offset_x:="{1}" _marker_offset_y:="{2}" _marker_offset_z:="{3}"'.format(self.package_id,
                                           self.marker_offset_x, self.marker_offset_y, self.marker_offset_z, self.cam2imuTf),
                                           remap_args=[('target_pose','package_pose')])
        self.launch_mt = roslaunch.scriptapi.ROSLaunch()
        self.launch_mt.start()


        self.node_ci = roslaunch.core.Node(package=self.package_ci, node_type=self.exe_ci, name='cam_info_pub',
                                           namespace=rospy.get_namespace())
        self.launch_ci = roslaunch.scriptapi.ROSLaunch()
        self.launch_ci.start()

        self.first_start = True

        self.process_ar_track = 0
        self.process_mt = 0
        self.process_ci = 0

    def mission_state_cb(self, msg):

        self.ar_track_state = msg.data

    def exit_cb(self, arg1 , arg2):
        if self.process_ar_track != 0:
            self.process_ar_track.stop()
        if self.process_mt != 0:
            self.process_mt.stop()
        self.launch_ar_track.stop()
        self.launch_mt.stop()
        rospy.signal_shutdown('Received sigterm ir sigint.')

    def has_changed(self, value, value_old):
        if value == value_old:
            return False
        else:
            return True

    def run(self):

        while not rospy.is_shutdown():

            self.rate.sleep()
            if self.has_changed(self.ar_track_state, self.ar_track_state_old):

                if self.ar_track_state > 0:
                    if self.first_start:
                        self.process_ar_track = self.launch_ar_track.launch(self.node_ar_track)
                        self.process_mt = self.launch_mt.launch(self.node_mt)
                        self.process_ci = self.launch_ci.launch(self.node_ci)
                        self.first_start = False
                    else:
                        self.process_ar_track.start()
                        self.process_mt.start()
                        self.process_ci.start()
                else:
                    self.process_ar_track.stop()
                    self.process_mt.stop()
                    self.process_ci.stop()

                self.ar_track_state_old = self.ar_track_state

            if self.ar_track_state == 0:
                self.detectionFlagPub.publish(Int16(0))


if __name__ == '__main__':

    node_ctl = NodeCtl()
    node_ctl.run()


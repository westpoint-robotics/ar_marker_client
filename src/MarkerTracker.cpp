/*
 * MarkerTracker.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: thaus
 */

#include "MarkerTracker.h"

MarkerTracker::MarkerTracker() {
	// TODO Auto-generated constructor stub
	cam2UAV << 	0.0, -1.0, 0.0, -0.006, //-0.003 - ako se optitrack marker ne pomice
				-1.0, 0.0, 0.0, 0.0231, //0.0231 - ako se optitrack marker ne pomice
				0.0, 0.0, -1.0, -0.0818, //-0.1148 - ako se optitrack marker ne pomice
				0.0, 0.0, 0.0, 1.0;

	UAV2GlobalFrame << 1.0, 0.0, 0.0, 0.0,
					   0.0, 1.0, 0.0, 0.0,
					   0.0, 0.0, 1.0, 0.0,
					   0.0, 0.0, 0.0, 1.0;

	std::cout<<cam2UAV<<std::endl;
}

MarkerTracker::~MarkerTracker() {
	// TODO Auto-generated destructor stub
}

void MarkerTracker::odometryCallback(const nav_msgs::Odometry &msg)
{

}

void MarkerTracker::ar_track_alvar_sub(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
	int i;
    ar_track_alvar_msgs::AlvarMarker marker;

	for(i = 0; i < msg->markers.size(); i++) {
		marker = msg->markers[i];
        /*
        if (marker.id == usv_id) {
			ROS_INFO("USV detected");
			pub_usv_pose.publish(marker.pose.pose);
		}
        */
        if (marker.id == target_id) {
            ROS_INFO("target detected");
            marker.pose.header.stamp = ros::Time::now();
			pub_target_pose.publish(marker.pose);
		}
		else {
			ROS_INFO("Detected unexpected marker id = %d", marker.id);
		}
	}
}

/*
 * MarkerTracker.h
 *
 *  Created on: Mar 4, 2014
 *      Author: thaus
 */

#ifndef MARKERTRACKER_H_
#define MARKERTRACKER_H_

#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarker.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

class MarkerTracker {
public:
	MarkerTracker();
	virtual ~MarkerTracker();
    void ar_track_alvar_sub(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

	void setPubTargetPose(ros::Publisher pubTargetPose) {
		pub_target_pose = pubTargetPose;
	}

	int getTargetId() const {
		return target_id;
	}

	void setTargetId(int targetId) {
		target_id = targetId;
	}

    /*
	void setPubUsvPose(ros::Publisher pubUsvPose) {
		pub_usv_pose = pubUsvPose;
	}

	int getUsvId() const {
		return usv_id;
	}

	void setUsvId(int usvId) {
		usv_id = usvId;
	}
    */

    //int usv_id;
	int target_id;
    //ros::Publisher pub_usv_pose;
	ros::Publisher pub_target_pose;
};

#endif /* MARKERTRACKER_H_ */

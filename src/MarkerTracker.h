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
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"
#include <eigen3/Eigen/Eigen>
#include <dynamic_reconfigure/server.h>
#include <ar_marker_client/MarkerOffsetConfig.h>
#include "yaml-cpp/yaml.h"
#include "geometry_msgs/PointStamped.h"
 #include <ros/package.h>
#include <sensor_msgs/Imu.h>

class MarkerTracker {
public:
    MarkerTracker();
    virtual ~MarkerTracker();
    void ar_track_alvar_sub(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry &msg);
    void imuCallback(const sensor_msgs::Imu &msg);
    void quaternion2euler(double *quaternion, double *euler);
    void getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
    	double *orientationEuler, double *position);
    void getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
    	double *angles);
    void LoadParameters(std::string file);

    //dynamic_reconfigure::Server<marker_tracker::MarkerOffsetConfig>::CallbackType params_call;

	void setPubTargetPose(ros::Publisher pubTargetPose) {
		pub_target_pose = pubTargetPose;
	}

    void setPubTargetPose_f(ros::Publisher pubTargetPose_f) {
        pub_target_pose_f = pubTargetPose_f;
    }

    void setPubDetectionFlag(ros::Publisher fPub)
    {
    	pubDetectionFlag = fPub;
    }

	int getTargetId() const {
		return target_id;
	}

	void setTargetId(int targetId) {
		target_id = targetId;
	}

    void setMarkerOffset(double *offset) {
        markerOffset[0] = offset[0];
        markerOffset[1] = offset[1];
        markerOffset[2] = offset[2];
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

    int first_meas;
    //int usv_id;
    int target_id;
    double qGlobalFrame[4], positionGlobalFrame[3], eulerGlobalFrame[3];
    double markerPosition[3], markerOrientation[3];
    double markerPositionOld[3];
    double markerOffset[3];
    double filt_const;
    Eigen::Matrix4d cam2UAV, UAV2GlobalFrame, markerTRMatrix, markerGlobalFrame;
    //ros::Publisher pub_usv_pose;
    ros::Publisher pub_target_pose, pubDetectionFlag;
    ros::Publisher pub_target_pose_f;
    geometry_msgs::PointStamped markerPointStamped;

};

#endif /* MARKERTRACKER_H_ */

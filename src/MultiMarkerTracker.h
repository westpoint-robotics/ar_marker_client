/*
 * MultiMarkerTracker.h
 *
 *  Created on: Mar 17, 2017
 *      Author: thaus
 */

#ifndef MULTIMARKERTRACKER_H_
#define MULTIMARKERTRACKER_H_

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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <boost/lexical_cast.hpp>
#include <stdio.h>


class MultiMarkerTracker {
public:

    MultiMarkerTracker();
    virtual ~MultiMarkerTracker();
    void ar_track_alvar_sub(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry &msg);
    void imuCallback(const sensor_msgs::Imu &msg);
    void softCallback(const geometry_msgs::TransformStamped &msg);
    void quaternion2euler(double *quaternion, double *euler);
    void getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
    	double *orientationEuler, double *position);
    void getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
    	double *angles);
    void LoadParameters(std::string file);

    bool isValidMarkerId(int marker_id);
    bool isMainMarker(int marker_id);
    bool allMarkerFramesAdded();
    int canAddNewFrames();
    void extractMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    geometry_msgs::PoseStamped getUavPoseFromMarker(ar_track_alvar_msgs::AlvarMarker marker);
    void addNewFrames();
    void estimateUavPoseFromMarkers();
    void publishStaticTransformsBetweenMarkers();
    void initUavPosePublishers(ros::NodeHandle &nh);
    bool isAlignedMarkerWithSoft();
    void setAlignedFlag(bool flag);
    void setUseSoftFlag(bool flag);
    bool newSoftData;
    bool newBaseMarkerData;

    //dynamic_reconfigure::Server<marker_tracker::MarkerOffsetConfig>::CallbackType params_call;

    void setPubTargetPose(ros::Publisher pubTargetPose);
    void setPubTargetPose_f(ros::Publisher pubTargetPose_f);
    void setPubDetectionFlag(ros::Publisher fPub);
    void setMarkerOffset(double *offset);
    void setMarkerIds(std::vector<int> marker_ids);
    void setCameraFrame(std::string camera_frame);
    void setPubMarker0(ros::Publisher pub);
    void setRateFiltVelocity(double velocity);
    void setRateFiltTime(double time);
    void setMinMarkerDetection(int detection_number);

    int first_meas;
    //int usv_id;
    int target_id;
    std::vector<int> marker_ids;
    double qGlobalFrame[4], positionGlobalFrame[3], eulerGlobalFrame[3];
    double markerPosition[3], markerOrientation[3];
    double markerPositionOld[3];
    double markerOffset[3];
    double filt_const;
    double rate_filt_max_velocity;
    double rate_filt_max_delta_time;
    geometry_msgs::PointStamped uav_position;
    geometry_msgs::PointStamped uav_position_filtered;
    geometry_msgs::PointStamped uav_position_filtered_old;
    std::map<int, geometry_msgs::PoseStamped> marker_poses;
    std::map<int, geometry_msgs::PoseStamped> marker_poses_old;
    std::map<int, geometry_msgs::PoseStamped> uav_pose;
    std::map<int, geometry_msgs::PoseStamped> uav_pose_old;
    std::map<int, geometry_msgs::PoseStamped> uav_relative_pose;
    std::map<int, std::string> marker_frames_corrected;
    std::map<int, std::string> marker_frames;
    std::map<int, bool> marker_detected;
    std::map<int, int> marker_detected_counter;
    std::map<int, bool> marker_frame_added;
    std::map<int, tf::StampedTransform> marker_transform_stamped;
    std::map<int, std::vector<tf::Transform> > marker_transforms;
    std::map<int, int> base_markers;
    int min_detection_count;
    std::string main_marker_frame;
    Eigen::Matrix4d cam2UAV, UAV2GlobalFrame, markerTRMatrix, markerGlobalFrame;
    ros::Publisher pub_target_pose, pubDetectionFlag;
    ros::Publisher pub_target_pose_f;
    ros::Publisher pub_marker0;
    std::map<int,ros::Publisher> uav_pose_publishers;
    std::map<int,ros::Publisher> uav_relative_pose_publishers;
    std::map<int,ros::Publisher> marker_corrected_pose_publishers;
    geometry_msgs::PointStamped markerPointStamped;
    std::string camera_frame;
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener;
    geometry_msgs::TransformStamped softData;
    bool alignedFlag;
    bool use_soft;

};

#endif /* MULTIMARKERTRACKER_H_ */

#ifndef KALMANTRACKER_H
#define KALMANTRACKER_H

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include "ar_track_alvar_msgs/AlvarMarker.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "ar_marker_client/KalmanConfig.h"
#include "std_msgs/Int16.h"


class KalmanTracker
{
public:
    KalmanTracker();
    ~KalmanTracker();

    int target_id;
    int ros_rate;
    double q[2];
    double r;
    double max_delay;
    float roll, pitch, yaw;
    float x[2];
    float x_cov[2][2];
    float y[2];
    float y_cov[2][2];
    float z[2];
    float z_cov[2][2];
    float pos_m[3], pos_m_old[3];
    float dt;
    bool new_meas;
    double time_old;

    ros::NodeHandle nh;
    ros::Publisher pub_target_pose;
    ros::Publisher pub_target_vel;
    ros::Publisher pub_target_pose_f;
    ros::Publisher pub_target_vel_f;
    ros::Publisher pub_marker_found;
    ros::Subscriber sub_marker;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist vel;

    dynamic_reconfigure::Server<marker_tracker::KalmanConfig> server;
    dynamic_reconfigure::Server<marker_tracker::KalmanConfig>::CallbackType params_call;

    void modelUpdate();
    void measureUpdate();
    geometry_msgs::Twist quat2euler(geometry_msgs::Quaternion);
    geometry_msgs::Pose compensate_angles(geometry_msgs::Pose);

    void ar_track_alvar_sub(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

};

#endif // KALMANTRACKER_H

#include "KalmanTracker.h"

// constructor
KalmanTracker::KalmanTracker()
{
    nh = ros::NodeHandle("~");
    pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
    pub_target_vel = nh.advertise<geometry_msgs::TwistStamped>("target_vel", 1);
    pub_target_pose_f = nh.advertise<geometry_msgs::PoseStamped>("target_pose_f", 1);
    pub_target_vel_f = nh.advertise<geometry_msgs::TwistStamped>("target_vel_f", 1);
    pub_marker_found = nh.advertise<std_msgs::Int16>("marker_found", 1);
    sub_marker = nh.subscribe("/ar_pose_marker", 1, &KalmanTracker::ar_track_alvar_sub, this);

    nh.param<int>("target_id", target_id, 7);
    nh.param<int>("rate", ros_rate, 30);
    nh.param<double>("pos_noise", q[0], 1);
    nh.param<double>("vel_noise", q[1], 10);
    nh.param<double>("measure_noise", r, 0.01);
    nh.param<double>("max_delay", max_delay, 0.3);

    dt = 1.0 / ros_rate;

    //q[0] = 1;
    //q[1] = 10;
    //r = 0.01;
    new_meas = false;
    pos_m[0] = 0;
    pos_m[1] = 0;
    pos_m[2] = 0;

    pos_m_old[0] = 0;
    pos_m_old[1] = 0;
    pos_m_old[2] = 0;

    time_old = ros::Time::now().toSec();

    x[0] = 0;
    x[1] = 0;
    x_cov[0][0] = 1;
    x_cov[0][1] = 0;
    x_cov[1][0] = 0;
    x_cov[1][1] = 1;

    y[0] = 0;
    y[1] = 0;
    y_cov[0][0] = 1;
    y_cov[0][1] = 0;
    y_cov[1][0] = 0;
    y_cov[1][1] = 1;

    z[0] = 0;
    z[1] = 0;
    z_cov[0][0] = 1;
    z_cov[0][1] = 0;
    z_cov[1][0] = 0;
    z_cov[1][1] = 1;
}

// destructor
KalmanTracker::~KalmanTracker()
{
}

// callback to marker topic
void KalmanTracker::ar_track_alvar_sub(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    int i;
    ar_track_alvar_msgs::AlvarMarker marker;
    std_msgs::Int16 msg_int;
    if (msg->markers.size() > 0) {
        for(i = 0; i < msg->markers.size(); i++) {
            marker = msg->markers[i];
            if (marker.id == target_id) {

                //ROS_INFO("target detected");
                geometry_msgs::TwistStamped vel;
                double t_now = ros::Time::now().toSec();
                marker.pose.header.stamp = ros::Time::now();

                // correct signs
                marker.pose.pose.position.y = -marker.pose.pose.position.y;

                /*
                  It seems that ar_track_alvar algorithm has changed the order of quaternion components.
                  The following order and signs have been determined experimentally
                qx = quat.z;
                qy = quat.w;
                qz = -quat.x;
                qw = quat.y;
                */
                float dummy;
                dummy = marker.pose.pose.orientation.x;
                marker.pose.pose.orientation.x = marker.pose.pose.orientation.z;
                marker.pose.pose.orientation.z = -dummy;
                dummy = marker.pose.pose.orientation.y;
                marker.pose.pose.orientation.y = marker.pose.pose.orientation.w;
                marker.pose.pose.orientation.w = dummy;

                pub_target_pose.publish(marker.pose);

                // compensate angles
                marker.pose.pose = compensate_angles(marker.pose.pose);
                pos_m[0] = marker.pose.pose.position.x;
                pos_m[1] = marker.pose.pose.position.y;
                pos_m[2] = marker.pose.pose.position.z;

                geometry_msgs::Twist twist = quat2euler(marker.pose.pose.orientation);
                roll = twist.angular.x;
                pitch = twist.angular.y;
                yaw = twist.angular.z;
                new_meas = true;

                vel.header.stamp = ros::Time::now();
                vel.twist.linear.x = (pos_m[0] - pos_m_old[0]) / (t_now - time_old);
                vel.twist.linear.y = (pos_m[1] - pos_m_old[1]) / (t_now - time_old);
                vel.twist.linear.z = (pos_m[2] - pos_m_old[2]) / (t_now - time_old);
                pub_target_vel.publish(vel);
                pos_m_old[0] = pos_m[0];
                pos_m_old[1] = pos_m[1];
                pos_m_old[2] = pos_m[2];
                time_old = t_now;

                msg_int.data = 1;
                pub_marker_found.publish(msg_int);
            }
            else if (marker.id == target_id) {
                //ROS_INFO("Target detected");
                //pub_target_pose.publish(marker.pose);
                msg_int.data = -1;
                pub_marker_found.publish(msg_int);
            }
            else {
                //ROS_INFO("Detected unexpected marker id = %d", marker.id);
                msg_int.data = -2;
                pub_marker_found.publish(msg_int);
            }
        }
    }
    else {
        msg_int.data = 0;
        pub_marker_found.publish(msg_int);
    }
}

// kalman filter - prediction step
void KalmanTracker::modelUpdate() {
    //ROS_INFO("pos,vel cov %.1f %.1f", q[0], q[1]);
    x[0] = x[0] + dt * x[1];
    x_cov[0][0] = x_cov[0][0] + dt * (x_cov[1][0] + x_cov[0][1]) + dt * dt * x_cov[1][1] + q[0];
    x_cov[0][1] = x_cov[0][1] + dt * x_cov[1][1];
    x_cov[1][0] = x_cov[1][0] + dt * x_cov[1][1];
    x_cov[1][1] = x_cov[1][1] + q[1];

    y[0] = y[0] + dt * y[1];
    y_cov[0][0] = y_cov[0][0] + dt * (y_cov[1][0] + y_cov[0][1]) + dt * dt * y_cov[1][1] + q[0];
    y_cov[0][1] = y_cov[0][1] + dt * y_cov[1][1];
    y_cov[1][0] = y_cov[1][0] + dt * y_cov[1][1];
    y_cov[1][1] = y_cov[1][1] + q[1];

    z[0] = z[0] + dt * z[1];
    z_cov[0][0] = z_cov[0][0] + dt * (z_cov[1][0] + z_cov[0][1]) + dt * dt * z_cov[1][1] + q[0];
    z_cov[0][1] = z_cov[0][1] + dt * z_cov[1][1];
    z_cov[1][0] = z_cov[1][0] + dt * z_cov[1][1];
    z_cov[1][1] = z_cov[1][1] + q[1];
}

// kalman filter - correction step
void KalmanTracker::measureUpdate() {
    //ROS_INFO("Measure cov %.1f", r);
    float sk, k1, k2, dk;
    sk = x_cov[0][0] + r;
    k1 = x_cov[0][0] / sk;
    k2 = x_cov[1][0] / sk;
    dk = pos_m[0] - x[0];
    x[0] = x[0] + k1 * dk;
    x[1] = x[1] + k2 * dk;
    x_cov[0][0] = (1 - k1) * x_cov[0][0];
    x_cov[0][1] = (1 - k1) * x_cov[0][1];
    x_cov[1][0] = -k2 * x_cov[0][0] + x_cov[1][0];
    x_cov[1][1] = -k2 * x_cov[0][1] + x_cov[1][1];

    sk = y_cov[0][0] + r;
    k1 = y_cov[0][0] / sk;
    k2 = y_cov[1][0] / sk;
    dk = pos_m[1] - y[0];
    y[0] = y[0] + k1 * dk;
    y[1] = y[1] + k2 * dk;
    y_cov[0][0] = (1 - k1) * y_cov[0][0];
    y_cov[0][1] = (1 - k1) * y_cov[0][1];
    y_cov[1][0] = -k2 * y_cov[0][0] + y_cov[1][0];
    y_cov[1][1] = -k2 * y_cov[0][1] + y_cov[1][1];

    sk = z_cov[0][0] + r;
    k1 = z_cov[0][0] / sk;
    k2 = z_cov[1][0] / sk;
    dk = pos_m[2] - z[0];
    z[0] = z[0] + k1 * dk;
    z[1] = z[1] + k2 * dk;
    z_cov[0][0] = (1 - k1) * z_cov[0][0];
    z_cov[0][1] = (1 - k1) * z_cov[0][1];
    z_cov[1][0] = -k2 * z_cov[0][0] + z_cov[1][0];
    z_cov[1][1] = -k2 * z_cov[0][1] + z_cov[1][1];

}

/*
   Method transforms quaternion to euler angles in radian (order 3-2-1, yaw - pitch - roll )
*/
geometry_msgs::Twist KalmanTracker::quat2euler(geometry_msgs::Quaternion quat) {

    geometry_msgs::Twist twist;
    twist = geometry_msgs::Twist();

    double qx, qy, qz, qw;

    qx = quat.x;
    qy = quat.y;
    qz = quat.z;
    qw = quat.w;

    twist.angular.z =  atan2(2 * (qx * qy + qw * qz), 1 - 2 * (qy * qy + qz * qz));
    twist.angular.y = asin(-2 * (qx * qz - qw * qy));
    twist.angular.x = atan2(2 * (qx * qw + qy * qz) , 1 - 2 * (qx * qx + qy * qy));

    return twist;
}

geometry_msgs::Pose KalmanTracker::compensate_angles(geometry_msgs::Pose pose) {
    float qx, qy, qz, qw, qxy, qxz, qxw, qyz, qyw, qzw;
    float qxx, qyy, qzz;
    float r11, r12, r13, r21, r22, r23, r31, r32, r33;

    qx = pose.orientation.x;
    qy = pose.orientation.y;
    qz = pose.orientation.z;
    qw = pose.orientation.w;

    qxx = qx * qx;
    qyy = qy * qy;
    qzz = qz * qz;
    qxy = qx * qy;
    qxz = qx * qz;
    qxw = qx * qw;
    qyz = qy * qz;
    qyw = qy * qw;
    qzw = qz * qw;

    r11 = 1 - 2 * (qyy + qzz);
    r12 = 2 * (qxy - qzw);
    r13 = 2 * (qyw + qxz);
    r21 = 2 * (qxy + qzw);
    r22 = 1 - 2 * (qxx + qzz);
    r23 = 2 * (qyz - qxw);
    r31 = 2 * (qxz - qyw );
    r32 = 2 * (qxw + qyz);
    r33 = 1 - 2 * (qxx + qyy);

    //ROS_INFO("\n R = %.2f %.2f %.2f \n %.2f %.2f %.2f \n %.2f %.2f %.2f \n", r11, r12, r13, r21, r22, r23, r31, r32, r33);

    geometry_msgs::Pose pose_new = geometry_msgs::Pose();
    pose_new.position.x = r11 * pose.position.x + r12 * pose.position.y + r13 * pose.position.z;
    pose_new.position.y = r21 * pose.position.x + r22 * pose.position.y + r23 * pose.position.z;
    pose_new.position.z = r31 * pose.position.x + r32 * pose.position.y + r33 * pose.position.z;
    pose_new.orientation = pose.orientation;

    return pose_new;

}


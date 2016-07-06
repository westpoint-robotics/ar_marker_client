#include "KalmanTracker.h"



KalmanTracker *ktracker;
void params_callback(marker_tracker::KalmanConfig &config, uint32_t level);

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "kalman_tracker_node");
  ros::NodeHandle n;

  ktracker = new KalmanTracker();

  ktracker->params_call = boost::bind(params_callback, _1, _2);
  ktracker->server.setCallback(ktracker->params_call);

  ROS_INFO("Rate = %d", ktracker->ros_rate);
  ros::Rate rate(ktracker->ros_rate);
  ROS_INFO("Waiting for first measurement");
  while (!ktracker->new_meas) {
      ros::spinOnce();
      rate.sleep();
  }

  ROS_INFO("Fetched measurement. Moving on!");
  ktracker->time_old = ros::Time::now().toSec();
  ktracker->measureUpdate();
  ktracker->new_meas = false;

  geometry_msgs::PoseStamped pose_stmp;
  geometry_msgs::TwistStamped vel_stmp;
  // Main loop.
  while (n.ok())
  {
      ros::spinOnce();
      // check if we received new measurements in last 0.35 sec
      if (ros::Time::now().toSec() - ktracker->time_old > ktracker->max_delay) {
          ROS_INFO("Not receiving measurements");
          ktracker->x[1] = 0;
          ktracker->y[1] = 0;
          ktracker->z[1] = 0;
      }
      else {
          ktracker->modelUpdate();
      }

      if (ktracker->new_meas) {
          ktracker->measureUpdate();
          ktracker->time_old = ros::Time::now().toSec();
          ktracker->new_meas = false;
      }

      pose_stmp.header.stamp = ros::Time::now();
      pose_stmp.pose.position.x = ktracker->x[0];
      pose_stmp.pose.position.y = ktracker->y[0];
      pose_stmp.pose.position.z = ktracker->z[0];
      pose_stmp.pose.orientation.x = ktracker->roll * 180 / M_PI;  // rad to degrees
      pose_stmp.pose.orientation.y = ktracker->pitch * 180 / M_PI;
      pose_stmp.pose.orientation.z = ktracker->yaw * 180 / M_PI;
      ktracker->pub_target_pose_f.publish(pose_stmp);


      vel_stmp.header.stamp = ros::Time::now();
      vel_stmp.twist.linear.x = ktracker->x[1];
      vel_stmp.twist.linear.y = ktracker->y[1];
      vel_stmp.twist.linear.z = ktracker->z[1];
      ktracker->pub_target_vel_f.publish(vel_stmp);

      rate.sleep();
  }

  delete ktracker;
  return 0;
}


// dynamic reconfigure server callback
void params_callback(marker_tracker::KalmanConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure request: q1, q2, r = %f %f %f ", config.pos_noise, config.vel_noise, config.measure_noise);

    ktracker->q[0] = config.pos_noise;
    ktracker->q[1] = config.vel_noise;
    ktracker->r = config.measure_noise;
    ktracker->max_delay = config.max_delay;
}

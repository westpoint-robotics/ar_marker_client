#include "MarkerTracker.h"


/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "marker_tracker");
  ros::NodeHandle n;

  MarkerTracker *mtracker = new MarkerTracker();
  // Declare variables that can be modified by launch file or command line.
  int marker_id_usv;
  int marker_id_target;

  ros::NodeHandle private_node_handle_("~");
  //private_node_handle_.param("marker_id_usv", marker_id_usv, int(1));
  private_node_handle_.param("marker_id_target", marker_id_target, int(2));

  ros::Subscriber sub_message = n.subscribe("ar_pose_marker", 1, &MarkerTracker::ar_track_alvar_sub, mtracker);
  ros::Subscriber odom_message = n.subscribe("/euroc3/msf_core/odometry", 1, &MarkerTracker::odometryCallback, mtracker);

  // Create a publisher and name the topic.
  //ros::Publisher pub_usv_pose = n.advertise<geometry_msgs::Pose>("usv_pose", 10);
  ros::Publisher pub_target_pose = n.advertise<geometry_msgs::PoseStamped>("ar_tracker/pose", 1);


  //mtracker->setPubUsvPose(pub_usv_pose);
  mtracker->setPubTargetPose(pub_target_pose);
  //mtracker->setUsvId(marker_id_usv);
  mtracker->setTargetId(marker_id_target);
  // Tell ROS how fast to run this node.
  ros::Rate r(20);

  ROS_INFO("Setting target marker id %d \n", marker_id_target);

  // Main loop.
  while (n.ok())
  {
	  ros::spinOnce();
	  r.sleep();
  }

  delete mtracker;
  return 0;
} // end main()


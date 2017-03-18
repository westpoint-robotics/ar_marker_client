#include "MultiMarkerTracker.h"


/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "multi_marker_tracker");
  ros::NodeHandle n;

  MultiMarkerTracker *mtracker = new MultiMarkerTracker();
  // Declare variables that can be modified by launch file or command line.
  int marker_id_usv;
  int marker_id_target;
  double markerOffset[3] = {0.0};
  std::string odometry_callback, cam2imuTf;
  std::string camera_frame("cam3");  // todo, add as a param
  std::vector<int> marker_ids;
  //todo read marker ids from rosparam
  marker_ids.push_back(10);
  marker_ids.push_back(5);

  std::string path = ros::package::getPath("ar_marker_client");
  ROS_INFO("Readinf ros params");
  ros::NodeHandle private_node_handle_("~");
  //private_node_handle_.param("marker_id_usv", marker_id_usv, int(1));
  private_node_handle_.param("marker_id_target", marker_id_target, int(2));
  private_node_handle_.param("marker_offset_x", markerOffset[0], double(0));
  private_node_handle_.param("marker_offset_y", markerOffset[1], double(0));
  private_node_handle_.param("marker_offset_z", markerOffset[2], double(0));
  private_node_handle_.param("odometry_callback", odometry_callback, std::string("/euroc3/msf_core/odometry"));
  private_node_handle_.param("cam2imuTf", cam2imuTf, std::string("/parameters/cam2imu.yaml"));

  std::cout << "Marker offset (x,y,z) " << "(" << markerOffset[0] << ","<<markerOffset[1] << "," << markerOffset[2] << ")" << std::endl;

  std::cout << path << std::endl;
  //mtracker->LoadParameters(path + cam2imuTf);

  ros::Subscriber sub_message = n.subscribe("ar_pose_marker", 1, &MultiMarkerTracker::ar_track_alvar_sub, mtracker);
  ros::Subscriber odom_message = n.subscribe(odometry_callback, 1, &MultiMarkerTracker::odometryCallback, mtracker);
  ros::Subscriber imu_message = n.subscribe("/euroc3/imu", 1, &MultiMarkerTracker::imuCallback, mtracker);

  // Create a publisher and name the topic.
  //ros::Publisher pub_usv_pose = n.advertise<geometry_msgs::Pose>("usv_pose", 10);
  ros::Publisher pub_target_pose = n.advertise<geometry_msgs::PointStamped>("ar_tracker/pose", 1);
  ros::Publisher pub_target_pose_f = n.advertise<geometry_msgs::PoseStamped>("ar_tracker/pose_f", 1);
  ros::Publisher pubDetectionFlag = n.advertise<std_msgs::Int16>("detection_flag", 1);

  ROS_INFO("Initializing.");

  mtracker->setMarkerOffset(markerOffset);
  mtracker->setMarkerIds(marker_ids);
  mtracker->setPubTargetPose(pub_target_pose);
  mtracker->setPubTargetPose_f(pub_target_pose_f);
  mtracker->setPubDetectionFlag(pubDetectionFlag);
  mtracker->setCameraFrame(camera_frame);
  ROS_INFO("Initializing publisher.");
  mtracker->initUavPosePublishers(n);

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


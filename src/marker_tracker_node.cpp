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
  double markerOffset[3] = {0.0};
  double mean_x = 0, mean_y = 0, mean_z = 0;
  double diff_x = 0, diff_y = 0, diff_z = 0;
  int v_length;
  int meas_number = 0;
  std::string odometry_callback, cam2imuTf;

  std::string path = ros::package::getPath("ar_marker_client");

  ros::NodeHandle private_node_handle_("~");
  //private_node_handle_.param("marker_id_usv", marker_id_usv, int(1));
  private_node_handle_.param("marker_id_target", marker_id_target, int(2));
  private_node_handle_.param("marker_offset_x", markerOffset[0], double(0));
  private_node_handle_.param("marker_offset_y", markerOffset[1], double(0));
  private_node_handle_.param("marker_offset_z", markerOffset[2], double(0));
  private_node_handle_.param("soft_data_vector_length", v_length, int(200));
  private_node_handle_.param("odometry_callback", odometry_callback, std::string("/euroc3/msf_core/odometry"));
  private_node_handle_.param("cam2imuTf", cam2imuTf, std::string("/parameters/cam2imu.yaml"));

  std::cout << "Marker offset (x,y,z) " << "(" << markerOffset[0] << ","<<markerOffset[1] << "," << markerOffset[2] << ")" << std::endl;

  //mtracker->LoadParameters(path + cam2imuTf);

  ros::Subscriber sub_message = n.subscribe("ar_pose_marker", 1, &MarkerTracker::ar_track_alvar_sub, mtracker);
  ros::Subscriber odom_message = n.subscribe(odometry_callback, 1, &MarkerTracker::odometryCallback, mtracker);
  ros::Subscriber imu_message = n.subscribe("/euroc3/imu", 1, &MarkerTracker::imuCallback, mtracker);
  ros::Subscriber soft_message = n.subscribe("/euroc3/soft/soft_transform", 1, &MarkerTracker::softCallback, mtracker);

  // Create a publisher and name the topic.
  //ros::Publisher pub_usv_pose = n.advertise<geometry_msgs::Pose>("usv_pose", 10);
  ros::Publisher pub_target_pose = n.advertise<geometry_msgs::PointStamped>("ar_tracker/pose_k", 1);
  ros::Publisher pub_target_pose_f = n.advertise<geometry_msgs::PoseStamped>("ar_tracker/pose_f", 1);
  ros::Publisher pubDetectionFlag = n.advertise<std_msgs::Int16>("detection_flag", 1);


  mtracker->setMarkerOffset(markerOffset);
  //mtracker->setPubUsvPose(pub_usv_pose);
  mtracker->setPubTargetPose(pub_target_pose);
  mtracker->setPubTargetPose_f(pub_target_pose_f);
  mtracker->setPubDetectionFlag(pubDetectionFlag);

  //mtracker->setUsvId(marker_id_usv);
  mtracker->setTargetId(marker_id_target);
  // Tell ROS how fast to run this node.
  ros::Rate r(20);

  ROS_INFO("Setting target marker id %d \n", marker_id_target);
mtracker->setAlignedFlag(true);
  // Main loop.
  while (n.ok())
  {
	  ros::spinOnce();

    ros::Duration timeDiff = mtracker->markerPointStamped.header.stamp - mtracker->softData.header.stamp;
    std::cout<<timeDiff<<std::endl;

    if (fabs(timeDiff.toSec()) < 0.06 && meas_number<v_length)
    {
      meas_number++;

      diff_x = mtracker->softData.transform.translation.x - mtracker->markerPointStamped.point.x;
      diff_y = mtracker->softData.transform.translation.y - mtracker->markerPointStamped.point.y;
      diff_z = mtracker->softData.transform.translation.z - mtracker->markerPointStamped.point.z;

      mean_x += diff_x/v_length;
      mean_y += diff_y/v_length;
      mean_z += diff_z/v_length;

      if (meas_number >= v_length){
        markerOffset[0] = mean_x;
        markerOffset[1] = mean_y;
        markerOffset[2] = mean_z;
        mtracker->setMarkerOffset(markerOffset);
        mtracker->setAlignedFlag(true);
        std::cout<<"x: "<<markerOffset[0]<<"y: "<<markerOffset[1]<<"z: "<<markerOffset[2]<<std::endl;
      }
    }

	  r.sleep();
  }

  delete mtracker;
  return 0;
} // end main()


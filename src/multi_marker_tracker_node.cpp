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
  int min_marker_detection;
  int transform_sample_num;
  double markerOffset[3] = {0.0};
  double rate_filt_velocity;
  double rate_filt_time;
  double mean_x = 0, mean_y = 0, mean_z = 0;
  double diff_x = 0, diff_y = 0, diff_z = 0;
  double filt_const;
  double average_filter_threshold = 1.0;
  int average_filter_size = 5;
  int pose_filter_size = 5;
  double pose_filter_threshold = 0.5;
  std::string odometry_callback, cam2imuTf;
  std::string camera_frame;  // todo, add as a param
  std::vector<int> marker_ids;
  int filter_length;
  int meas_number = 0;
  bool use_optitrack_align;
  //todo read marker ids from rosparam
  //marker_ids.push_back(12);
  //marker_ids.push_back(10);
  //marker_ids.push_back(5);
  //marker_ids.push_back(11);

  std::string path = ros::package::getPath("ar_marker_client");
  ROS_INFO("Reading ros params");
  ros::NodeHandle private_node_handle_("~");
  //private_node_handle_.param("marker_id_usv", marker_id_usv, int(1)); 
  //private_node_handle_.param("odometry_callback", odometry_callback, std::string("/euroc3/msf_core/odometry"));
  private_node_handle_.getParam("cam2imuTf", cam2imuTf);
  private_node_handle_.getParam("marker_ids", marker_ids);
  private_node_handle_.getParam("rate_filt_velocity", rate_filt_velocity);
  private_node_handle_.getParam("rate_filt_time", rate_filt_time);
  private_node_handle_.getParam("camera_frame", camera_frame);
  private_node_handle_.getParam("min_marker_detection", min_marker_detection);
  private_node_handle_.getParam("marker_transform_sample_num", transform_sample_num);
  private_node_handle_.param("optitrack_data_vector_length", filter_length, int(100));
  private_node_handle_.param("use_optitrack_align", use_optitrack_align, false);
  private_node_handle_.param("filter_const", filt_const, 0.9);
  private_node_handle_.param("average_filter_size", average_filter_size, 5);
  private_node_handle_.param("average_filter_threshold", average_filter_threshold, 1.0);
  private_node_handle_.param("pose_filter_size", pose_filter_size, 5);
  private_node_handle_.param("pose_filter_threshold", pose_filter_threshold, 0.5);



  ROS_INFO("Number of marker ids %d", (int)marker_ids.size());

  for(int i = 0; i < marker_ids.size(); i++) {
      ROS_INFO("Marker %d, id %d", i, marker_ids[i]);
  }

  ROS_INFO("Parameters, %s, %s, %.2f, %.2f,  %d ", cam2imuTf.c_str(), camera_frame.c_str(), rate_filt_velocity, rate_filt_time, min_marker_detection);
  ROS_INFO("Average filter parameters, %d, %.2f, pose average filter parameters %d %.2f", 
    average_filter_size, average_filter_threshold, pose_filter_size, pose_filter_threshold);

  //std::cout << "Marker offset (x,y,z) " << "(" << markerOffset[0] << ","<<markerOffset[1] << "," << markerOffset[2] << ")" << std::endl;

  //std::cout << path << std::endl;
  mtracker->LoadParameters(path + cam2imuTf);

  ros::Subscriber sub_message = n.subscribe("ar_pose_marker", 1, &MultiMarkerTracker::ar_track_alvar_sub, mtracker);
  //ros::Subscriber odom_message = n.subscribe(odometry_callback, 1, &MultiMarkerTracker::odometryCallback, mtracker);
  ros::Subscriber imu_message = n.subscribe("imu", 1, &MultiMarkerTracker::imuCallback, mtracker);
  ros::Subscriber soft_message = n.subscribe("optitrack/pose", 1, &MultiMarkerTracker::optitrackCallback, mtracker);

  // Create a publisher and name the topic.
  //ros::Publisher pub_usv_pose = n.advertise<geometry_msgs::Pose>("usv_pose", 10);
  ros::Publisher pub_target_pose = n.advertise<geometry_msgs::PointStamped>("ar_tracker/pose", 1);
  ros::Publisher pub_target_pose_f = n.advertise<geometry_msgs::PointStamped>("ar_tracker/pose_f", 1);
  ros::Publisher pub_target_transform = n.advertise<geometry_msgs::TransformStamped>("ar_tracker/pose_tf", 1);
  ros::Publisher pub_marker0 = n.advertise<geometry_msgs::PoseStamped>("ar_tracker/marker0", 1);
  ros::Publisher pubDetectionFlag = n.advertise<std_msgs::Int16>("ar_tracker/detection_flag", 1);

  ROS_INFO("Initializing.");

  //mtracker->setOptitrackToMarkerTransform(markerOffset);
  mtracker->setMarkerIds(marker_ids);
  mtracker->setPubTargetPose(pub_target_pose);
  mtracker->setPubTargetPose_f(pub_target_pose_f);
  mtracker->setPubTargetTransform(pub_target_transform);
  mtracker->setPubMarker0(pub_marker0);
  mtracker->setPubDetectionFlag(pubDetectionFlag);
  mtracker->setCameraFrame(camera_frame);
  mtracker->setRateFiltTime(rate_filt_time);
  mtracker->setRateFiltVelocity(rate_filt_velocity);
  mtracker->setMinMarkerDetection(min_marker_detection);
  mtracker->setUseOptitrackFlag(use_optitrack_align);
  mtracker->setFilterConst(filt_const);
  mtracker->setMarkerTransformSampleNum(transform_sample_num);
  mtracker->setAverageFilterParameters(average_filter_size, average_filter_threshold);
  mtracker->setPoseAverageFilterParameters(pose_filter_size, pose_filter_threshold);
  ROS_INFO("Initializing publisher.");
  mtracker->initUavPosePublishers(n);

  //mtracker->setUsvId(marker_id_usv);
  //mtracker->setTargetId(marker_id_target);
  // Tell ROS how fast to run this node.
  ros::Rate r(100);

  //ROS_INFO("Setting target marker id %d \n", marker_id_target);

  // Main loop.
  while (n.ok())
  {
      ros::spinOnce();

      //ROS_INFO("Main loop.");

      tf::Transform uavOptitrackPose;
      tf::Transform uavMarkerPose;
      tf::Transform optitrackToMarker;
      std::vector<tf::Transform> optitrackToMarkerTransforms;

      if (mtracker->use_optitrack_align) {
        if (mtracker->newOptitrackData && mtracker->newBaseMarkerData) {

          ros::Duration timeDiff = mtracker->uav_relative_pose[marker_ids[0]].header.stamp - mtracker->optitrackData.header.stamp;

          if (fabs(timeDiff.toSec()) < 0.16 && meas_number<filter_length) {
              meas_number++;
              //std::cout<<meas_number<<std::endl;

              //tf::transformMsgToTF(mtracker->optitrackData.transform, uavOptitrackPose);
              uavOptitrackPose.setOrigin(tf::Vector3(mtracker->optitrackData.transform.translation.x,
                                                  mtracker->optitrackData.transform.translation.y,
                                                  mtracker->optitrackData.transform.translation.z));
              uavOptitrackPose.setRotation(tf::Quaternion(mtracker->optitrackData.transform.rotation.x,
                                                       mtracker->optitrackData.transform.rotation.y,
                                                       mtracker->optitrackData.transform.rotation.z,
                                                       mtracker->optitrackData.transform.rotation.w));

              //uavOptitrackPose.setRotation(tf::Quaternion(0,0,0,1));

              uavMarkerPose.setOrigin(tf::Vector3(mtracker->uav_relative_pose[marker_ids[0]].pose.position.x,
                                                  mtracker->uav_relative_pose[marker_ids[0]].pose.position.y,
                                                  mtracker->uav_relative_pose[marker_ids[0]].pose.position.z));
              uavMarkerPose.setRotation(tf::Quaternion(mtracker->uav_relative_pose[marker_ids[0]].pose.orientation.x,
                                                       mtracker->uav_relative_pose[marker_ids[0]].pose.orientation.y,
                                                       mtracker->uav_relative_pose[marker_ids[0]].pose.orientation.z,
                                                       mtracker->uav_relative_pose[marker_ids[0]].pose.orientation.w));

              optitrackToMarkerTransforms.push_back(uavOptitrackPose * uavMarkerPose.inverse());

              tf::Matrix3x3 m(uavOptitrackPose.getRotation());
              double roll, pitch, yaw;
              m.getRPY(roll, pitch, yaw);
              ROS_INFO("Optitrack2UAV pos, rot %.2f %.2f %.2f %.2f, %.2f, %.2f", uavOptitrackPose.getOrigin().getX(), uavOptitrackPose.getOrigin().getY(),uavOptitrackPose.getOrigin().getZ(),
                       roll, pitch, yaw);

              m = tf::Matrix3x3(uavMarkerPose.getRotation());
              m.getRPY(roll, pitch, yaw);
              ROS_INFO("Marker2UAV pos, rot %.2f %.2f %.2f %.2f, %.2f, %.2f", uavMarkerPose.getOrigin().getX(), uavMarkerPose.getOrigin().getY(),uavMarkerPose.getOrigin().getZ(),
                       roll, pitch, yaw);
              ROS_INFO("-----------------------------------------------");


              /*
              diff_x = mtracker->optitrackData.transform.translation.x - mtracker->uav_relative_pose[marker_ids[0]].pose.position.x;
              diff_y = mtracker->optitrackData.transform.translation.y - mtracker->uav_relative_pose[marker_ids[0]].pose.position.y;
              diff_z = mtracker->optitrackData.transform.translation.z - mtracker->uav_relative_pose[marker_ids[0]].pose.position.z;

              mean_x += diff_x/filter_length;
              mean_y += diff_y/filter_length;
              mean_z += diff_z/filter_length;
              */
              if (meas_number >= filter_length){
                /*

                markerOffset[0] = mean_x;
                markerOffset[1] = mean_y;
                markerOffset[2] = mean_z;
                */

                  tf::Vector3 position(0,0,0);
                  tf::Quaternion rotation(0,0,0,1);

                  // average position and orientation

                  int transform_samples_num = optitrackToMarkerTransforms.size();

                  position  = optitrackToMarkerTransforms[0].getOrigin() / transform_samples_num;
                  rotation  = optitrackToMarkerTransforms[0].getRotation();

                  for(int j = 1; j < transform_samples_num ; j++) {

                      position = position + optitrackToMarkerTransforms[j].getOrigin() / transform_samples_num;
                      rotation = rotation.slerp(optitrackToMarkerTransforms[j].getRotation(), 1 / (j + 1));
                  }

                  optitrackToMarker.setOrigin(position);
                  optitrackToMarker.setRotation(rotation);
                  //optitrackToMarker.setRotation(tf::Quaternion(0,0,0,1));

                  mtracker->setOptitrackToMarkerTransform(optitrackToMarker);
                  mtracker->setAlignedFlag(true);

                  tf::Matrix3x3 m(optitrackToMarker.getRotation());
                  double roll, pitch, yaw;
                  m.getRPY(roll, pitch, yaw);
                  ROS_INFO("Optitrack to marker transform, position %.2f %.2f %.2f", optitrackToMarker.getOrigin().getX(), optitrackToMarker.getOrigin().getY(),optitrackToMarker.getOrigin().getZ());
                  ROS_INFO("Optitrack to marker transform, roll, pitch, yaw %.2f, %.2f, %.2f", roll, pitch, yaw );
              }
          }
          mtracker->newOptitrackData = false;
          mtracker->newBaseMarkerData = false;
        }
      }
      r.sleep();
  }

  delete mtracker;
  return 0;
} // end main()


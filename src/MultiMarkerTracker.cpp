/*
 * MultiMarkerTracker.cpp
 *
 *  Created on: Mar 17, 2017
 *      Author: thaus
 */

#include "MultiMarkerTracker.h"

MultiMarkerTracker::MultiMarkerTracker() {
    // TODO Auto-generated constructor stub

    cam2UAV << 	0.0, -1.0, 0.0, 0.005, //-0.003 - ako se optitrack marker ne pomice
                -1.0, 0.0, 0.0, -0.007, //0.0231 - ako se optitrack marker ne pomice
                0.0, 0.0, -1.0, -0.05148, //-0.1148 - ako se optitrack marker ne pomice
                0.0, 0.0, 0.0, 1.0;
                
    UAV2GlobalFrame << 1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.0, 0.0, 0.0, 1.0;

    filt_const = 0.9;
    markerPositionOld[0] = 0;
    markerPositionOld[1] = 0;
    markerPositionOld[2] = 0;
    first_meas = 0;
    min_detection_count = 10;
}

MultiMarkerTracker::~MultiMarkerTracker() {
	// TODO Auto-generated destructor stub
}

void MultiMarkerTracker::LoadParameters(std::string file)
{
  // First open .yaml file
  YAML::Node config = YAML::LoadFile(file);
  std::vector<double> cam2imu_vector;
  cam2imu_vector = config["cam2imu"].as<std::vector<double> >();
  cam2UAV << cam2imu_vector[0], cam2imu_vector[1], cam2imu_vector[2], cam2imu_vector[3], //-0.003 - ako se optitrack marker ne pomice
             cam2imu_vector[4], cam2imu_vector[5], cam2imu_vector[6], cam2imu_vector[7], //0.0231 - ako se optitrack marker ne pomice
             cam2imu_vector[8], cam2imu_vector[9], cam2imu_vector[10], cam2imu_vector[11], //-0.1148 - ako se optitrack marker ne pomice
             cam2imu_vector[12], cam2imu_vector[13], cam2imu_vector[14], cam2imu_vector[15];
}

void MultiMarkerTracker::quaternion2euler(double *quaternion, double *euler)
{
  euler[0] = atan2(2 * (quaternion[0] * quaternion[1] + 
    quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1]
    + quaternion[2] * quaternion[2]));

  euler[1] = asin(2 * (quaternion[0] * quaternion[2] -
    quaternion[3] * quaternion[1]));

  euler[2] = atan2(2 * (quaternion[0]*quaternion[3] +
    quaternion[1]*quaternion[2]), 1 - 2 * (quaternion[2]*quaternion[2] +
    quaternion[3] * quaternion[3]));
}

void MultiMarkerTracker::getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
  double *orientationEuler, double *position)
{
	double r11, r12, r13, t1, r21, r22, r23, t2;
	double r31, r32, r33, t3;

	double x, y, z;

  	x = orientationEuler[0];
  	y = orientationEuler[1];
  	z = orientationEuler[2];

	r11 = cos(y)*cos(z);

  r12 = cos(z)*sin(x)*sin(y) - cos(x)*sin(z);

	r13 = sin(x)*sin(z) + cos(x)*cos(z)*sin(y);

	r21 = cos(y)*sin(z);

	r22 = cos(x)*cos(z) + sin(x)*sin(y)*sin(z);

	r23 = cos(x)*sin(y)*sin(z) - cos(z)*sin(x);

	r31 = -sin(y);

	r32 = cos(y)*sin(x);

	r33 = cos(x)*cos(y);

	t1 = position[0];
	t2 = position[1];
	t3 = position[2];


	rotationTranslationMatrix << r11, r12, r13, t1,
				     r21, r22, r23, t2,
	                             r31, r32, r33, t3,
	                             0,   0,   0,   1;
}

void MultiMarkerTracker::getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
 double *angles)
{
  double r11, r21, r31, r32, r33;
  double roll, pitch, yaw;

  r11 = rotationTranslationMatrix(0,0);
  r21 = rotationTranslationMatrix(1,0);
  r31 = rotationTranslationMatrix(2,0);
  r32 = rotationTranslationMatrix(2,1);
  r33 = rotationTranslationMatrix(2,2);

  roll = atan2(r32, r33);
  pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
  yaw = atan2(r21, r11);

  angles[0] = roll;
  angles[1] = pitch;
  angles[2] = yaw;
}

void MultiMarkerTracker::imuCallback(const sensor_msgs::Imu &msg)
{
  qGlobalFrame[1] = msg.orientation.x;
  qGlobalFrame[2] = msg.orientation.y;
  qGlobalFrame[3] = msg.orientation.z;
  qGlobalFrame[0] = msg.orientation.w;
  positionGlobalFrame[0] = 0;
  positionGlobalFrame[1] = 0;
  positionGlobalFrame[2] = 0;

  quaternion2euler(qGlobalFrame, eulerGlobalFrame);

  getRotationTranslationMatrix(UAV2GlobalFrame, eulerGlobalFrame, positionGlobalFrame);
}

void MultiMarkerTracker::odometryCallback(const nav_msgs::Odometry &msg)
{
}

void MultiMarkerTracker::ar_track_alvar_sub(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {

  int i;
  ar_track_alvar_msgs::AlvarMarker marker;
  geometry_msgs::PointStamped markerPointStamped;
  std_msgs::Int16 detection_flag_msg;
  // publish tfs
  tf::Transform tf_transform;

  for (int i = 0; i < marker_ids.size(); i++) {
      marker_detected[marker_ids[i]] = false;
  }
  for(i = 0; i < msg->markers.size(); i++) {
          marker = msg->markers[i];

    if (isValidMarkerId(marker.id)) {
        //ROS_INFO("Detected marker id %d", marker.id);

        correctMarkerPose(marker);
        marker_detected[marker.id] = true;
        marker_detected_counter[marker.id]++;
        //ROS_INFO("Corrected position marker id %d, %.2f, %.2f, %.2f", marker.id, marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z);


        tf_transform.setOrigin( tf::Vector3(marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z));
        tf_transform.setRotation( tf::Quaternion(marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w ));
        tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), camera_frame.c_str(),  marker_frames_corrected[marker_ids[i]].c_str()));

        //publish za poseition_update
        /*
        markerPointStamped.header.stamp = ros::Time::now();
        markerPointStamped.header.frame_id = "fcu";
        markerPointStamped.point.x = marker.pose.pose.position.x;
        markerPointStamped.point.y = marker.pose.pose.position.y;
        markerPointStamped.point.z = marker.pose.pose.position.z;
        pub_target_pose.publish(markerPointStamped);

        //pub_target_pose.publish(marker.pose);
        detection_flag_msg.data = 1;
        pubDetectionFlag.publish(detection_flag_msg);
        packageDetectedFlag = true;

        if (first_meas == 0) {
            first_meas = 1;
            markerPositionOld[0] = marker.pose.pose.position.x;
            markerPositionOld[1] = marker.pose.pose.position.y;
            markerPositionOld[2] = marker.pose.pose.position.z;
        }

        marker.pose.pose.position.x = filt_const * markerPositionOld[0] + (1-filt_const) * marker.pose.pose.position.x;
        marker.pose.pose.position.y = filt_const * markerPositionOld[1] + (1-filt_const) * marker.pose.pose.position.y;
        marker.pose.pose.position.z = filt_const * markerPositionOld[2] + (1-filt_const) * marker.pose.pose.position.z;
        pub_target_pose_f.publish(marker.pose);
        markerPositionOld[0] = marker.pose.pose.position.x;
        markerPositionOld[1] = marker.pose.pose.position.y;
        markerPositionOld[2] = marker.pose.pose.position.z;
        */
    }
    else {
            ROS_INFO("Detected unexpected marker id = %d", marker.id);
    }
  }


  if (!allMarkerFramesAdded()) {

      int marker_base_frame_id = canAddNewFrames();

      if (marker_base_frame_id >= 0) {
          std::string str("[id, count, detected] = ");
          for (std::vector<int>::size_type i=0; i < marker_ids.size(); i++) {

              if (!marker_frame_added[marker_ids[i]]) {
                  if (isMainMarker(marker_ids[i])) {
                    marker_frame_added[marker_ids[i]] = true;
                  }
                  else if (marker_detected[marker_ids[i]] &&  (marker_detected_counter[marker_ids[i]] > min_detection_count)) {
                    // add tf betweer marker and main marker frame
                      tf::StampedTransform transform_stamped;
                      try {
                        tf_listener.lookupTransform(marker_frames_corrected[marker_base_frame_id].c_str(), marker_frames_corrected[marker_ids[i]].c_str(),
                        ros::Time(0), transform_stamped);
                      }
                      catch (tf::TransformException &ex) {
                         ROS_ERROR("%s",ex.what());
                         continue;
                      }

                      tf_transform.setOrigin( transform_stamped.getOrigin());
                      tf_transform.setRotation( transform_stamped.getRotation());
                      tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), marker_frames[marker_base_frame_id].c_str(), marker_frames[marker_ids[i]].c_str()));

                      marker_frame_added[marker_ids[i]] = true;
                      ROS_INFO("Adding marker frame %d", marker_ids[i]);
                  }

              }
              str += std::string("[") + boost::lexical_cast<std::string>(marker_ids[i]) + std::string(",")
                  + boost::lexical_cast<std::string>(marker_detected_counter[marker_ids[i]]) + std::string(",")
                  + boost::lexical_cast<std::string>(marker_frame_added[marker_ids[i]]) + std::string("]; ");
        }
        ROS_INFO("%s", str.c_str());
    }
  }

  tf::StampedTransform marker_to_marker_transform, marker_to_uav_transform;
  tf::Transform uav_to_marker_transform;
  tf::Vector3 uav_to_marker_position;
  tf::Quaternion uav_to_marker_orientation;

  for(int i = 0; i < marker_ids.size(); i++) {

      if (marker_detected[marker_ids[i]] && marker_frame_added[marker_ids[i]]) {

          tf_listener.lookupTransform(camera_frame.c_str(), marker_frames_corrected[marker_ids[i]].c_str(),ros::Time(0), marker_to_uav_transform);
          uav_to_marker_transform = marker_to_uav_transform.inverse();

          tf_listener.lookupTransform(marker_frames[marker_ids[0]].c_str(), marker_frames[marker_ids[i]], ros::Time(0), marker_to_marker_transform);
          uav_to_marker_position = marker_to_marker_transform(uav_to_marker_transform.getOrigin());
          uav_to_marker_orientation = uav_to_marker_transform.getRotation() * marker_to_marker_transform.getRotation();

          uav_pose[marker_ids[i]].pose.position.x = uav_to_marker_position.getX();
          uav_pose[marker_ids[i]].pose.position.y = uav_to_marker_position.getY();
          uav_pose[marker_ids[i]].pose.position.z = uav_to_marker_position.getZ();
          tf::quaternionTFToMsg(uav_to_marker_orientation, uav_pose[marker_ids[i]].pose.orientation);
          uav_pose[marker_ids[i]].header.stamp = ros::Time::now();
          uav_pose_publishers[marker_ids[i]].publish(uav_pose[marker_ids[i]]);
      }

  }


  /*
  if(packageDetectedFlag == false)
  {
      detection_flag_msg.data = 0;
      pubDetectionFlag.publish(detection_flag_msg);
  }
  */
}

bool MultiMarkerTracker::isValidMarkerId(int marker_id) {
  return (std::find(marker_ids.begin(), marker_ids.end(), marker_id) != marker_ids.end());
}

bool MultiMarkerTracker::isMainMarker(int marker_id) {
  return (std::find(marker_ids.begin(), marker_ids.end(), marker_id) == marker_ids.begin());
}

int MultiMarkerTracker::canAddNewFrames() {
    for (int i = 0; i < marker_ids.size(); i++) {
        if (marker_detected[marker_ids[i]]) {
          if (isMainMarker(marker_ids[i]) || marker_frame_added[marker_ids[i]])
            return marker_ids[i];
        }
    }
    return -1;
}

bool MultiMarkerTracker::allMarkerFramesAdded() {
  bool res = true;
  for (int i = 0; i < marker_ids.size(); i++) {
      res &= marker_frame_added[marker_ids[i]];
  }
  return res;
}

void MultiMarkerTracker::initUavPosePublishers(ros::NodeHandle &nh) {
  for(int i = 0; i < marker_ids.size(); i++) {
     uav_pose_publishers[marker_ids[i]] =  nh.advertise<geometry_msgs::PoseStamped>((std::string("uav_pose") + boost::lexical_cast<std::string>(i)).c_str(), 1);
  }
  ROS_INFO("Initialized pose publishers");
}

void MultiMarkerTracker::correctMarkerPose(ar_track_alvar_msgs::AlvarMarker marker) {
  double q_marker[4], euler_marker[3];

  markerPosition[0] = marker.pose.pose.position.x;
  markerPosition[1] = marker.pose.pose.position.y;
  markerPosition[2] = marker.pose.pose.position.z;

  q_marker[1] = marker.pose.pose.orientation.x;
  q_marker[2] = marker.pose.pose.orientation.y;
  q_marker[3] = marker.pose.pose.orientation.z;
  q_marker[0] = marker.pose.pose.orientation.w;

  quaternion2euler(q_marker, euler_marker);

  markerOrientation[0] = euler_marker[0];
  markerOrientation[1] = euler_marker[1];
  markerOrientation[2] = euler_marker[2];

  getRotationTranslationMatrix(markerTRMatrix, markerOrientation, markerPosition);
  markerGlobalFrame = UAV2GlobalFrame * cam2UAV * markerTRMatrix;

  //getAnglesFromRotationTranslationMatrix(markerGlobalFrame, markerOrientation);
  Eigen::Matrix3d rotation_matrix = markerGlobalFrame.block<3,3>(0,0);
  Eigen::Quaterniond quaternion(rotation_matrix);

  marker.pose.pose.position.x = markerGlobalFrame(0,3);
  marker.pose.pose.position.y = markerGlobalFrame(1,3);
  marker.pose.pose.position.z = markerGlobalFrame(2,3);

  /*
  marker.pose.pose.position.x += (markerOffset[0])*cos(markerOrientation[2]) - (markerOffset[1])*sin(markerOrientation[2]);
  marker.pose.pose.position.y +=  (markerOffset[0])*sin(markerOrientation[2]) + (markerOffset[1])*cos(markerOrientation[2]);
  marker.pose.pose.position.z += markerOffset[2];
  */

  marker.pose.pose.orientation.x = quaternion.x();
  marker.pose.pose.orientation.y = quaternion.y();
  marker.pose.pose.orientation.z = quaternion.z();
  marker.pose.pose.orientation.w = quaternion.w();

  marker.pose.header.stamp = ros::Time::now();
  marker.pose.header.frame_id = "fcu";
}

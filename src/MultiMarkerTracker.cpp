/*
 * MultiMarkerTracker.cpp
 *
 *  Created on: Mar 17, 2017
 *      Author: thaus
 */

#include "MultiMarkerTracker.h"

MultiMarkerTracker::MultiMarkerTracker() {
    // TODO Auto-generated constructor stub

    uav2cam << 	0.0, 1.0, 0.0, 0.005, //-0.003 - ako se optitrack marker ne pomice
                -1.0, 0.0, 0.0, -0.007, //0.0231 - ako se optitrack marker ne pomice
                0.0, 0.0, 1.0, -0.05148, //-0.1148 - ako se optitrack marker ne pomice
                0.0, 0.0, 0.0, 1.0;

    cam2marker << 1.0, 0.0, 0.0, 0.0,
             0.0, -1.0, 0.0, 0.0,
             0.0, 0.0, -1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;

    inertial2uav << 1.0, 0.0, 0.0, 0.0,
                     0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0,
                     0.0, 0.0, 0.0, 1.0;

    filt_const = 0.9;
    markerPositionOld[0] = 0;
    markerPositionOld[1] = 0;
    markerPositionOld[2] = 0;
    first_meas = 0;
    min_detection_count = 15;
    rate_filt_max_velocity = 1;  // m/s
    rate_filt_max_delta_time = 0.1;
    alignedFlag = false;
    newSoftData = false;
    newBaseMarkerData = false;
    use_soft = false;
    marker_transform_samples_num = 20;

}

MultiMarkerTracker::~MultiMarkerTracker() {
	// TODO Auto-generated destructor stub
}

void MultiMarkerTracker::LoadParameters(std::string file)
{
  // First open .yaml file
  /*
  YAML::Node config = YAML::LoadFile(file);
  std::vector<double> cam2imu_vector;
  cam2imu_vector = config["cam2imu"].as<std::vector<double> >();
  cam2UAV << cam2imu_vector[0], cam2imu_vector[1], cam2imu_vector[2], cam2imu_vector[3], //-0.003 - ako se optitrack marker ne pomice
             cam2imu_vector[4], cam2imu_vector[5], cam2imu_vector[6], cam2imu_vector[7], //0.0231 - ako se optitrack marker ne pomice
             cam2imu_vector[8], cam2imu_vector[9], cam2imu_vector[10], cam2imu_vector[11], //-0.1148 - ako se optitrack marker ne pomice
             cam2imu_vector[12], cam2imu_vector[13], cam2imu_vector[14], cam2imu_vector[15];
  */
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
  eulerGlobalFrame[2] = 0.0; //set imu yaw to 0

  getRotationTranslationMatrix(inertial2uav, eulerGlobalFrame, positionGlobalFrame);
}

void MultiMarkerTracker::odometryCallback(const nav_msgs::Odometry &msg)
{
}

void MultiMarkerTracker::softCallback(const geometry_msgs::TransformStamped &msg)
{
  //double soft_q[4], soft_euler[3];

  //soft_q = msg.

  //quaternion2euler(soft_q, soft_euler);

  softData = msg;
  newSoftData = true;
  //soft_yaw = soft_euler[2];
}

bool MultiMarkerTracker::isAlignedMarkerWithSoft()
{
  return alignedFlag;
}

void MultiMarkerTracker::setAlignedFlag(bool flag)
{
  alignedFlag = flag;
}

void MultiMarkerTracker::setPubTargetPose(ros::Publisher pubTargetPose) {
        pub_target_pose = pubTargetPose;
}

void MultiMarkerTracker::setPubTargetPose_f(ros::Publisher pubTargetPose_f) {
    pub_target_pose_f = pubTargetPose_f;
}

void MultiMarkerTracker::setPubDetectionFlag(ros::Publisher fPub)
{
    pubDetectionFlag = fPub;
}


void MultiMarkerTracker::setMarkerOffset(double *offset) {
    markerOffset[0] = offset[0];
    markerOffset[1] = offset[1];
    markerOffset[2] = offset[2];
}

void MultiMarkerTracker::setMarkerIds(std::vector<int> marker_ids) {
  for(std::vector<int>::size_type i = 0; i != marker_ids.size(); i++) {
      this->marker_ids.push_back(marker_ids[i]);
      this->marker_detected_counter[this->marker_ids[i]] = 0;
      this->marker_frame_added[this->marker_ids[i]] = false;
      marker_frames[this->marker_ids[i]] = std::string("marker")
                                           + boost::lexical_cast<std::string>(marker_ids[i]);
      marker_frames_corrected[this->marker_ids[i]] = std::string("ar_marker_")
                                                     + boost::lexical_cast<std::string>(marker_ids[i])
                                                     + std::string("_corrected");
  }
}

void MultiMarkerTracker::setCameraFrame(std::string camera_frame) {
  this->camera_frame = camera_frame;
}

void MultiMarkerTracker::setPubMarker0(ros::Publisher pub) {
    pub_marker0 = pub;
 }

void MultiMarkerTracker::setRateFiltVelocity(double velocity) {
    rate_filt_max_velocity = velocity;
}

void MultiMarkerTracker::setRateFiltTime(double time) {
    rate_filt_max_delta_time = time;
}

void MultiMarkerTracker::setMinMarkerDetection(int detection_number) {
    min_detection_count = detection_number;
}

bool MultiMarkerTracker::isValidMarkerId(int marker_id) {
  //return (std::find(marker_ids.begin(), marker_ids.end(), marker_id) != marker_ids.end());
  for(int i=0; i < marker_ids.size(); i++) {
      if (marker_ids[i] == marker_id)
        return true;
  }
  return false;
}

bool MultiMarkerTracker::isMainMarker(int marker_id) {
  //return (std::find(marker_ids.begin(), marker_ids.end(), marker_id) == marker_ids.begin());
  return (marker_ids[0] == marker_id);
}

int MultiMarkerTracker::getBaseMarker() {
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
     uav_relative_pose_publishers[marker_ids[i]] = nh.advertise<geometry_msgs::PoseStamped>((std::string("uav_pose_relative") + boost::lexical_cast<std::string>(i)).c_str(), 1);
     marker_corrected_pose_publishers[marker_ids[i]] = nh.advertise<geometry_msgs::PoseStamped>((std::string("marker_corrected") + boost::lexical_cast<std::string>(i)).c_str(), 1);
  }
  ROS_INFO("Initialized pose publishers");
}

void MultiMarkerTracker::setUseSoftFlag(bool flag) {
  use_soft = flag;
}

void MultiMarkerTracker::setMarkerTransformSampleNum(int samples_num) {
  marker_transform_samples_num = samples_num;
}




void MultiMarkerTracker::ar_track_alvar_sub(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {

  publishStaticTransformsBetweenMarkers();

  for (int i = 0; i < marker_ids.size(); i++) {
      marker_detected[marker_ids[i]] = false;
  }

  extractMarkers(msg);

  if (!allMarkerFramesAdded()) {

      addNewFrames();
  }

  estimateUavPoseFromMarkers();

}




void MultiMarkerTracker::extractMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {

  ar_track_alvar_msgs::AlvarMarker marker;

  for(int i = 0; i < msg->markers.size(); i++) {
          marker = msg->markers[i];

     if (isValidMarkerId(marker.id)) {
        //ROS_INFO("Detected marker id %d", marker.id);

        if (isMainMarker(marker.id)) {

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.pose.position.x = marker.pose.pose.position.x;
            pose_msg.pose.position.y = marker.pose.pose.position.y;
            pose_msg.pose.position.z = marker.pose.pose.position.z;

           /*
            pose_msg.pose.orientation.x = marker.pose.pose.orientation.x;
            pose_msg.pose.orientation.y = marker.pose.pose.orientation.y;
            pose_msg.pose.orientation.z = marker.pose.pose.orientation.z;
            pose_msg.pose.orientation.w = marker.pose.pose.orientation.w;
      */
            double q_marker[4], euler_marker[3];

            q_marker[1] = marker.pose.pose.orientation.x;
            q_marker[2] = marker.pose.pose.orientation.y;
            q_marker[3] = marker.pose.pose.orientation.z;
            q_marker[0] = marker.pose.pose.orientation.w;

            quaternion2euler(q_marker, euler_marker);

            pose_msg.pose.orientation.x = euler_marker[0];
            pose_msg.pose.orientation.y = euler_marker[1];
            pose_msg.pose.orientation.z = euler_marker[2];
            pose_msg.pose.orientation.w = 0;

            pose_msg.header.stamp = ros::Time::now();
            pub_marker0.publish(pose_msg);

        }

        geometry_msgs::PoseStamped uav_pose;
        uav_pose =  getUavPoseFromMarker(marker);
        marker_detected[marker.id] = true;
        marker_detected_counter[marker.id]++;
        //ROS_INFO("Corrected position marker id %d, %.2f, %.2f, %.2f", marker.id, marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z);

        // publish uav pose relative to the marker
        //ROS_INFO("Publish uav pose relative to marker %d", marker.id);
        uav_relative_pose_publishers[marker.id].publish(uav_pose);
        uav_relative_pose[marker.id] = uav_pose;

        if (isMainMarker(marker.id)) {
            newBaseMarkerData = true;
        }

        // publish corrected transform from cam to the detected marker
        tf::Transform marker_to_uav, uav_to_marker;
        marker_to_uav.setOrigin( tf::Vector3(uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z));
        marker_to_uav.setRotation( tf::Quaternion(uav_pose.pose.orientation.x, uav_pose.pose.orientation.y, uav_pose.pose.orientation.z, uav_pose.pose.orientation.w ));
        //ROS_INFO("Camera frame %s, marker frame %s", camera_frame.c_str(), marker_frames_corrected[marker.id].c_str());
        uav_to_marker = marker_to_uav.inverse();
        tf_broadcaster.sendTransform(tf::StampedTransform(uav_to_marker, ros::Time::now(), camera_frame.c_str(),  marker_frames_corrected[marker.id].c_str()));

        // publish corrected marker
        geometry_msgs::PoseStamped marker_pose;
        marker_pose.pose.position.x = uav_to_marker.getOrigin().getX();
        marker_pose.pose.position.y = uav_to_marker.getOrigin().getY();
        marker_pose.pose.position.z = uav_to_marker.getOrigin().getZ();
        marker_pose.pose.orientation.x = uav_to_marker.getRotation().getX();
        marker_pose.pose.orientation.y = uav_to_marker.getRotation().getY();
        marker_pose.pose.orientation.z = uav_to_marker.getRotation().getZ();
        marker_pose.pose.orientation.w = uav_to_marker.getRotation().getW();
        marker_pose.header.stamp = marker.header.stamp;
        marker_corrected_pose_publishers[marker.id].publish(marker_pose);

    }
    else {
            //ROS_INFO("Detected unexpected marker id = %d", marker.id);
    }
  }

}

geometry_msgs::PoseStamped MultiMarkerTracker::getUavPoseFromMarker(ar_track_alvar_msgs::AlvarMarker marker) {

  // input ar marker - position of the marker w.r.t the UAV camera
  // output pose stamped - position of the UAV w.r.t. the marker frame

  double q_marker[4], euler_marker[3];
  geometry_msgs::PoseStamped uav_pose;

  q_marker[1] = marker.pose.pose.orientation.x;
  q_marker[2] = marker.pose.pose.orientation.y;
  q_marker[3] = marker.pose.pose.orientation.z;
  q_marker[0] = marker.pose.pose.orientation.w;

  quaternion2euler(q_marker, euler_marker);

  markerOrientation[0] = euler_marker[0];
  markerOrientation[1] = euler_marker[1];
  markerOrientation[2] = euler_marker[2];

  markerPosition[0] = marker.pose.pose.position.x;
  markerPosition[1] = marker.pose.pose.position.y;
  markerPosition[2] = marker.pose.pose.position.z;

  getRotationTranslationMatrix(markerTRMatrix, markerOrientation, markerPosition);

  inertial2marker = inertial2uav * uav2cam * cam2marker * markerTRMatrix;
  marker2inertial = inertial2marker.inverse();

  getAnglesFromRotationTranslationMatrix(marker2inertial, markerOrientation);

  uav_pose.pose.position.x = marker2inertial(0,3);
  uav_pose.pose.position.y = marker2inertial(1,3);
  uav_pose.pose.position.z = marker2inertial(2,3);

  if (isAlignedMarkerWithSoft()) {
    uav_pose.pose.position.x += markerOffset[0];
    uav_pose.pose.position.y += markerOffset[1];
    uav_pose.pose.position.z += markerOffset[2];
  }


  Eigen::Matrix3d rotation_matrix = marker2inertial.block<3,3>(0,0);
  Eigen::Quaterniond quaternion(rotation_matrix);
  uav_pose.pose.orientation.x = quaternion.x();
  uav_pose.pose.orientation.y = quaternion.y();
  uav_pose.pose.orientation.z = quaternion.z();
  uav_pose.pose.orientation.w = quaternion.w();

  uav_pose.header.stamp = marker.header.stamp;
  uav_pose.header.frame_id = "world";

  return uav_pose;

}

void MultiMarkerTracker::addNewFrames() {

  tf::Transform tf_transform;

  for (int i = 0; i < marker_ids.size(); i++) {

      if ((!marker_frame_added[marker_ids[i]]) && marker_detected[marker_ids[i]]) {

          //ROS_INFO("Marker %d detected but its frame not yet added!", marker_ids[i]);

          if (isMainMarker(marker_ids[i])) {

              // publish zero transform to cam3
              tf::Vector3 zero_origin(0,0,0);
              tf::Quaternion unit_quaternion(0,0,0,1);
              tf_transform.setOrigin( zero_origin);
              tf_transform.setRotation( unit_quaternion);
              tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), camera_frame.c_str(), marker_frames[marker_ids[i]].c_str()));

              tf::StampedTransform tf_stamped;
              tf_stamped.setData(tf_transform);
              marker_transform_stamped[marker_ids[i]].setData(tf_transform);
              marker_transform_stamped[marker_ids[i]].frame_id_ = camera_frame.c_str();
              marker_transform_stamped[marker_ids[i]].child_frame_id_ = marker_frames[marker_ids[i]].c_str();
              marker_frame_added[marker_ids[i]] = true;
              ROS_INFO("Adding marker frame %d", marker_ids[i]);
          }
          else {

              //ROS_INFO("Considering marker %d", marker_ids[i]);

              if (marker_base_frames.find(marker_ids[i]) == marker_base_frames.end()) {
                 // base frame not set for marker i
                 //ROS_INFO("Base frame for marker %d not added", marker_ids[i]);

                 int marker_base_frame_id = getBaseMarker();

                 if (marker_base_frame_id >= 0) {
                     // set marker base for marker
                     ROS_INFO("For marker %d adding base marker %d", marker_ids[i], marker_base_frame_id);
                     marker_base_frames[marker_ids[i]] = marker_base_frame_id;
                 }

              }
              else {
                  // base frame for the marker already set, try to find transform

                  //ROS_INFO("Base frame for marker %d already added", marker_ids[i]);

                  if (marker_detected[marker_base_frames[marker_ids[i]]]) {

                      // add tf betweer marker and main marker frame
                      tf::StampedTransform transform_stamped;
                      try {
                        tf_listener.lookupTransform(marker_frames_corrected[marker_base_frames[marker_ids[i]]].c_str(), marker_frames_corrected[marker_ids[i]].c_str(),
                        ros::Time(0), transform_stamped);
                      }
                      catch (tf::TransformException &ex) {
                         ROS_ERROR("%s",ex.what());
                         continue;
                      }

                      tf_transform.setOrigin( transform_stamped.getOrigin());
                      tf_transform.setRotation( transform_stamped.getRotation());

                      marker_transforms[marker_ids[i]].push_back(tf_transform);

                      if (marker_transforms[marker_ids[i]].size() == marker_transform_samples_num) {

                          ROS_INFO("Collected %d samples for transform between marker %d and marker %d", marker_transform_samples_num, marker_base_frames[marker_ids[i]], marker_ids[i]);
                          tf::Transform marker_transform;
                          tf::Vector3 position(0,0,0);
                          tf::Quaternion rotation(0,0,0,1);

                          // average position and orientation

                          int transform_samples_num = marker_transforms[marker_ids[i]].size();


                          position  = marker_transforms[marker_ids[i]][0].getOrigin() / transform_samples_num;
                          rotation  = marker_transforms[marker_ids[i]][0].getRotation();

                          for(int j = 1; j < transform_samples_num ; j++) {

                             position = position + marker_transforms[marker_ids[i]][j].getOrigin() / transform_samples_num;
                             rotation = rotation.slerp(marker_transforms[marker_ids[i]][j].getRotation(), 1 / (j + 1));
                          }


                          marker_transform.setOrigin( position);
                          marker_transform.setRotation( rotation);
                          tf_broadcaster.sendTransform(tf::StampedTransform(marker_transform, ros::Time::now(), marker_frames[marker_base_frames[marker_ids[i]]].c_str(), marker_frames[marker_ids[i]].c_str()));

                          marker_transform_stamped[marker_ids[i]].setData(marker_transform);
                          marker_transform_stamped[marker_ids[i]].frame_id_ =  marker_frames[marker_base_frames[marker_ids[i]]].c_str();
                          marker_transform_stamped[marker_ids[i]].child_frame_id_ = marker_frames[marker_ids[i]].c_str();
                          marker_frame_added[marker_ids[i]] = true;
                          ROS_INFO("Adding marker frame %d", marker_ids[i]);

                          tf::Matrix3x3 m(marker_transform.getRotation());
                          double roll, pitch, yaw;
                          m.getRPY(roll, pitch, yaw);

                          ROS_INFO("Adding child frame %s with parent %s, translation %.2f %.2f %.2f, rpy %.2f %.2f %.2f",
                                   marker_frames_corrected[marker_ids[i]].c_str(), marker_frames_corrected[marker_base_frames[marker_ids[i]]].c_str(),
                                   marker_transform.getOrigin().getX(), marker_transform.getOrigin().getY(), marker_transform.getOrigin().getZ(),
                                   roll, pitch, yaw);

                      }
                  }
              }
          }
      }
   }
}

void MultiMarkerTracker::estimateUavPoseFromMarkers() {
  tf::StampedTransform marker_to_marker_transform;
  tf::Transform uav_to_marker_transform, marker_to_base_marker_tf, uav_to_base_marker_tf;

  uav_position.point.x = 0;
  uav_position.point.y = 0;
  uav_position.point.z = 0;
  uav_position_filtered.point.x = 0;
  uav_position_filtered.point.y = 0;
  uav_position_filtered.point.z = 0;
  int markers_detected = 0;
  int filtered_markers_detected = 0;

  for(int i = 0; i < marker_ids.size(); i++) {

      if (marker_detected[marker_ids[i]] && marker_frame_added[marker_ids[i]]) {

          try {
            tf_listener.lookupTransform(marker_frames[marker_ids[0]].c_str(), marker_frames[marker_ids[i]],  ros::Time(0), marker_to_marker_transform);

            marker_to_base_marker_tf.setOrigin(marker_to_marker_transform.getOrigin());
            marker_to_base_marker_tf.setRotation(marker_to_marker_transform.getRotation());
            uav_to_marker_transform.setOrigin(tf::Vector3(uav_relative_pose[marker_ids[i]].pose.position.x,
                                                          uav_relative_pose[marker_ids[i]].pose.position.y,
                                                          uav_relative_pose[marker_ids[i]].pose.position.z));
            uav_to_marker_transform.setRotation(tf::Quaternion(uav_relative_pose[marker_ids[i]].pose.orientation.x,
                                                               uav_relative_pose[marker_ids[i]].pose.orientation.y,
                                                               uav_relative_pose[marker_ids[i]].pose.orientation.z,
                                                               uav_relative_pose[marker_ids[i]].pose.orientation.w));

            uav_to_base_marker_tf = marker_to_base_marker_tf * uav_to_marker_transform;
            uav_pose[marker_ids[i]].pose.position.x = uav_to_base_marker_tf.getOrigin().getX();
            uav_pose[marker_ids[i]].pose.position.y = uav_to_base_marker_tf.getOrigin().getY();
            uav_pose[marker_ids[i]].pose.position.z = uav_to_base_marker_tf.getOrigin().getZ();
            uav_pose[marker_ids[i]].pose.orientation.x = uav_to_base_marker_tf.getRotation().getX();
            uav_pose[marker_ids[i]].pose.orientation.y = uav_to_base_marker_tf.getRotation().getY();
            uav_pose[marker_ids[i]].pose.orientation.z = uav_to_base_marker_tf.getRotation().getZ();
            uav_pose[marker_ids[i]].pose.orientation.w = uav_to_base_marker_tf.getRotation().getW();
            uav_pose[marker_ids[i]].header.stamp = uav_relative_pose[marker_ids[i]].header.stamp;
            uav_pose_publishers[marker_ids[i]].publish(uav_pose[marker_ids[i]]);

            uav_position.point.x += uav_pose[marker_ids[i]].pose.position.x;
            uav_position.point.y += uav_pose[marker_ids[i]].pose.position.y;
            uav_position.point.z += uav_pose[marker_ids[i]].pose.position.z;
            uav_position.header.stamp =  uav_pose[marker_ids[i]].header.stamp;
            markers_detected++;

            // rate filter
            if (first_meas == 0) {
                uav_position_filtered_old.point.x = uav_pose[marker_ids[i]].pose.position.x;
                uav_position_filtered_old.point.y = uav_pose[marker_ids[i]].pose.position.y;
                uav_position_filtered_old.point.z = uav_pose[marker_ids[i]].pose.position.z;
                uav_position_filtered_old.header.stamp = uav_pose[marker_ids[i]].header.stamp;
                first_meas = 1;
            }
            else {

                double dt = (uav_pose[marker_ids[i]].header.stamp.toSec() - uav_position_filtered_old.header.stamp.toSec());
                // do the filtering only if get continuous markers
                //if (dt < rate_filt_max_delta_time) {
                double max_dl = dt * rate_filt_max_velocity;
                if ((fabs( uav_pose[marker_ids[i]].pose.position.x - uav_position_filtered_old.point.x) < max_dl) &&
                    (fabs( uav_pose[marker_ids[i]].pose.position.y - uav_position_filtered_old.point.y) < max_dl) &&
                    (fabs( uav_pose[marker_ids[i]].pose.position.z - uav_position_filtered_old.point.z) < max_dl)) {
                    // valid marker
                    uav_position_filtered.point.x += uav_pose[marker_ids[i]].pose.position.x;
                    uav_position_filtered.point.y += uav_pose[marker_ids[i]].pose.position.y;
                    uav_position_filtered.point.z += uav_pose[marker_ids[i]].pose.position.z;
                    uav_position_filtered.header.stamp = uav_pose[marker_ids[i]].header.stamp;
                    filtered_markers_detected++;

                }
                else {
                    ROS_INFO("Ignoring measurement from marker %d", marker_ids[i]);
                }

              //}
                /*
              else {
               // we receive the marker pose after a while, just use it in measurement
                  uav_position_filtered.point.x += uav_pose[marker_ids[i]].pose.position.x;
                  uav_position_filtered.point.y += uav_pose[marker_ids[i]].pose.position.y;
                  uav_position_filtered.point.z += uav_pose[marker_ids[i]].pose.position.z;
                  uav_position_filtered.header.stamp = uav_pose[marker_ids[i]].header.stamp;
                  filtered_markers_detected++;
              }
              */

            }



          }
          catch (tf::TransformException &ex) {
             ROS_ERROR("%s",ex.what());
             continue;
          }

          //ROS_INFO("Publishing transfor for marker id %d", marker_ids[i]);

      }

  }

  if (markers_detected > 0) {
    uav_position.point.x = uav_position.point.x / markers_detected;
    uav_position.point.y = uav_position.point.y / markers_detected;
    uav_position.point.z = uav_position.point.z / markers_detected;

    if (use_soft) {
      if (isAlignedMarkerWithSoft()) {
        pub_target_pose.publish(uav_position);
      }
    }
    else {
        pub_target_pose.publish(uav_position);
    }


  }

  //ROS_INFO("Markers %d, filtered markers %d", markers_detected, filtered_markers_detected);

  if (filtered_markers_detected > 0) {
    uav_position_filtered.point.x = uav_position_filtered.point.x / filtered_markers_detected;
    uav_position_filtered.point.y = uav_position_filtered.point.y / filtered_markers_detected;
    uav_position_filtered.point.z = uav_position_filtered.point.z / filtered_markers_detected;

    if (use_soft) {
      if (isAlignedMarkerWithSoft()) {
         pub_target_pose_f.publish(uav_position_filtered);
      }
    }
    else {
        pub_target_pose_f.publish(uav_position_filtered);
    }
    uav_position_filtered_old = uav_position_filtered;

  }
}

void MultiMarkerTracker::publishStaticTransformsBetweenMarkers() {
  // publish tranform between markers
  for (int i = 0; i < marker_ids.size(); i++) {
      if (marker_frame_added[marker_ids[i]]) {
        marker_transform_stamped[marker_ids[i]].stamp_ = ros::Time::now();
        tf_broadcaster.sendTransform(marker_transform_stamped[marker_ids[i]]);
      }
  }
}

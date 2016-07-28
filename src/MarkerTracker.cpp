/*
 * MarkerTracker.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: thaus
 */

#include "MarkerTracker.h"

MarkerTracker::MarkerTracker() {
    // TODO Auto-generated constructor stub

    cam2UAV << 	0.0, -1.0, 0.0, -0.006, //-0.003 - ako se optitrack marker ne pomice
                -1.0, 0.0, 0.0, 0.0231, //0.0231 - ako se optitrack marker ne pomice
                0.0, 0.0, -1.0, -0.0815, //-0.1148 - ako se optitrack marker ne pomice
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
}

MarkerTracker::~MarkerTracker() {
	// TODO Auto-generated destructor stub
}

void MarkerTracker::LoadParameters(std::string file)
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

void MarkerTracker::quaternion2euler(double *quaternion, double *euler)
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

void MarkerTracker::getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
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

void MarkerTracker::getAnglesFromRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix,
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

void MarkerTracker::odometryCallback(const nav_msgs::Odometry &msg)
{
	qGlobalFrame[1] = msg.pose.pose.orientation.x;
	qGlobalFrame[2] = msg.pose.pose.orientation.y;
	qGlobalFrame[3] = msg.pose.pose.orientation.z;
	qGlobalFrame[0] = msg.pose.pose.orientation.w;
    positionGlobalFrame[0] = msg.pose.pose.position.x - markerOffset[0];
    positionGlobalFrame[1] = msg.pose.pose.position.y - markerOffset[1];
    positionGlobalFrame[2] = msg.pose.pose.position.z - markerOffset[2];

	quaternion2euler(qGlobalFrame, eulerGlobalFrame);

	getRotationTranslationMatrix(UAV2GlobalFrame, eulerGlobalFrame, positionGlobalFrame);

}

void MarkerTracker::ar_track_alvar_sub(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
	int i;
    ar_track_alvar_msgs::AlvarMarker marker;

  bool packageDetectedFlag = false;
	for(i = 0; i < msg->markers.size(); i++) {
		marker = msg->markers[i];
        /*
        if (marker.id == usv_id) {
			ROS_INFO("USV detected");
			pub_usv_pose.publish(marker.pose.pose);
		}
        */
        if (marker.id == target_id) {
            ROS_INFO("target detected");

            markerPosition[0] = marker.pose.pose.position.x;
            markerPosition[1] = marker.pose.pose.position.y;
            markerPosition[2] = marker.pose.pose.position.z;

            markerOrientation[0] = marker.pose.pose.orientation.x;
            markerOrientation[1] = marker.pose.pose.orientation.y;
            markerOrientation[2] = marker.pose.pose.orientation.z;

            getRotationTranslationMatrix(markerTRMatrix, markerOrientation, markerPosition);
            markerGlobalFrame = UAV2GlobalFrame * cam2UAV * markerTRMatrix;

            getAnglesFromRotationTranslationMatrix(markerGlobalFrame, markerOrientation);

            marker.pose.pose.position.x = markerGlobalFrame(0,3);
            marker.pose.pose.position.y = markerGlobalFrame(1,3);
            marker.pose.pose.position.z = markerGlobalFrame(2,3);

            marker.pose.pose.orientation.x = markerOrientation[0];
            marker.pose.pose.orientation.y = markerOrientation[1];
            marker.pose.pose.orientation.z = markerOrientation[2];

            marker.pose.header.stamp = ros::Time::now();
			      pub_target_pose.publish(marker.pose);
            pubDetectionFlag.publish(1);
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
        }
		else {
			ROS_INFO("Detected unexpected marker id = %d", marker.id);
		}
	}
  if(packageDetectedFlag==false)
  {
    pubDetectionFlag.publish(0);
  }
}

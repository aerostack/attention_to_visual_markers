#ifndef QR_CODE_LOCALIZER_H
#define QR_CODE_LOCALIZER_H

#include <string>
#include <queue>
#include <array>
#include <cmath>
// ROS
#include <ros/ros.h>
#include "cv.h"
#include "highgui.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <armadillo>    
#include <queue>         

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <aerostack_msgs/QrCodeLocalized.h>
#include <aerostack_msgs/ListOfQrCodeLocalized.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <image_transport/image_transport.h>
#include <boost/bind.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/CameraInfo.h>
#include <thread>
#include <mutex>
// Aerostack
#include <robot_process.h>
#include <droneMsgsROS/QRInterpretation.h>
#include <stdio.h>
#include <math.h>
#include "zbar.h"
#include "std_msgs/Bool.h"

using namespace zbar;

class QrCodeLocalizer : public RobotProcess
{

  public:
    QrCodeLocalizer();
    ~QrCodeLocalizer();
      void ownSetUp();
      void ownRun();
      void ownStop();
      void ownStart();


  private:

	ros::NodeHandle node_handle;
	cv_bridge::CvImagePtr image_cv;
	sensor_msgs::CameraInfo camera_info;
  	geometry_msgs::PoseStamped selfPose;
	geometry_msgs::Vector3 angles;
	droneMsgsROS::QRInterpretation interpretation;
	float mx;
	float area;
	bool subscriptions_complete;

    std::string drone_id;
    std::string drone_console_interface_sensor_front_camera;
    std::mutex mtx;


  //Congfig variables

  	std::string camera_topic_str;
  	std::string camera_info_topic_str;
  	std::string pose_topic_str;
	std::string notification_topic;
	std::string qr_position_topic_str;

	geometry_msgs::Vector3 ToEulerAngles(const geometry_msgs::Quaternion q);
  	void CameraCallback (const sensor_msgs::ImageConstPtr& msg);
  	void CameraInfoCallback (const sensor_msgs::CameraInfo& msg);
  	void PoseCallback (const geometry_msgs::PoseStamped &msg);
	void QrCallback (const droneMsgsROS::QRInterpretation &msg);
    	void imagesFrontReceptionCallback(const sensor_msgs::ImageConstPtr& msg);
	aerostack_msgs::ListOfQrCodeLocalized list_points;
	ImageScanner scanner;
    	bool entered;

	ros::Publisher qr_code_localized_pub;
	ros::Publisher notification_pub;

  	ros::Subscriber camera_info_sub;
  	ros::Subscriber camera_sub;
  	ros::Subscriber pose_sub;

};
#endif

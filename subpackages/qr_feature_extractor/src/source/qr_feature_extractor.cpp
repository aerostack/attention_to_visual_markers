/*!*******************************************************************************************
 *  \file       QrFeatureExtractor.cpp
 *  \brief      distance measurement implementation file.
 *  \details    This file contains the DistanceMeasurement implementattion of the class.
 *  \authors    Javier Melero Deza
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All Rights Reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#include "../include/qr_feature_extractor.h"
#include <pluginlib/class_list_macros.h>


QrFeatureExtractor::QrFeatureExtractor(){
}

QrFeatureExtractor::~QrFeatureExtractor(){
}

void QrFeatureExtractor::ownSetUp(){
    ros::NodeHandle nh("~");

    nh.param<std::string>("camera_topic", camera_topic_str, "usb_cam/image_raw");
    nh.param<std::string>("camera_info_topic", camera_info_topic_str, "usb_cam/camera_info");
    nh.param<std::string>("estimated_pose_topic", pose_topic_str, "self_localization/pose");
    nh.param<std::string>("qr_interpretation_topic", notification_topic,  "qr_interpretation");
    nh.param<std::string>("qr_position_topic", qr_position_topic_str,  "qr_code_localized");

    
    camera_sub = node_handle.subscribe(camera_topic_str, 30, &QrFeatureExtractor::CameraCallback, this);
    camera_info_sub = node_handle.subscribe(camera_info_topic_str, 30, &QrFeatureExtractor::CameraInfoCallback, this);
    pose_sub = node_handle.subscribe(pose_topic_str, 1, &QrFeatureExtractor::PoseCallback, this);
    qr_code_localized_pub = node_handle.advertise<aerostack_msgs::ListOfQrCodeLocalized>(qr_position_topic_str, 30, true);
    notification_pub = node_handle.advertise<droneMsgsROS::QRInterpretation>(notification_topic, 30, true);
    

}

void QrFeatureExtractor::PoseCallback (const geometry_msgs::PoseStamped &msg){
    selfPose = msg;
    angles = ToEulerAngles(selfPose.pose.orientation); //Obtain the drone orientation and turn in into radians.
    
}

void QrFeatureExtractor::CameraCallback (const sensor_msgs::ImageConstPtr &msg){ 

  mtx.lock();
  try 
  {
    image_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // ToDo := If mutex implementation doesn't unlock upon destruction or destruction doesn't happen
  // when going through the catch clause, there will be a deadlock
  mtx.unlock();
  if(image_cv)
    {
      // Check for invalid input
      int width = image_cv->image.cols;
      int height = image_cv->image.rows;
      uchar *raw = (uchar *)image_cv->image.data;
      Image image(width, height, "Y800", raw, width * height);
      // scan the image for barcodes
      int n = scanner.scan(image);
      int loc_size = 0;
      Image::SymbolIterator symbol = image.symbol_begin();

      entered=false;
      // extract results
      for(Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end();
        ++symbol) {
                   entered=true;
                   std::vector<cv::Point> vp;
        // do something useful with results
        interpretation.message= symbol->get_data();
        for(int i = 0; i < interpretation.message.length(); i++)
        {
          if(interpretation.message[i] == ' ')
          interpretation.message[i] = '_';
         }

        notification_pub.publish(interpretation);
        aerostack_msgs::QrCodeLocalized point;
        if(interpretation.message != "") {    
      // extract results
            // get location for each qr symbol
            loc_size = symbol->get_location_size();
            int loc_x1 = symbol->get_location_x(0);
            int loc_x2 = symbol->get_location_x(1);
            int loc_x3 = symbol->get_location_x(2);
            int loc_x4 = symbol->get_location_x(3);
            int loc_y1 = symbol->get_location_y(0);
            int loc_y2 = symbol->get_location_y(1);
            int loc_y3 = symbol->get_location_y(2);
            int loc_y4 = symbol->get_location_y(3);

            std::vector<cv::Point2f> points (4);

            points[0] = cv::Point2f(float(loc_x1), float(loc_y1));
            points[1] = cv::Point2f(float(loc_x2), float(loc_y2));
            points[2] = cv::Point2f(float(loc_x3), float(loc_y3));
            points[3] = cv::Point2f(float(loc_x4), float(loc_y4));

            cv::RotatedRect box = cv::minAreaRect(points); // assign bounding coordinates to points variable
            // get bounding box for qr symbol
            geometry_msgs::Point p0;
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;
            geometry_msgs::Point p3;
            p0.x = float(loc_x1);
            p0.y = float(loc_y1);
            p0.z = 0.0;
            p1.x = float(loc_x2);
            p1.y = float(loc_y2);
            p1.z = 0.0;
            p2.x = float(loc_x3);
            p2.y = float(loc_y3);
            p2.z = 0.0;
            p3.x = float(loc_x4);
            p3.y = float(loc_y4);
            p3.z = 0.0;
            
            point.bounding_points.push_back(p0);
            point.bounding_points.push_back(p1);
            point.bounding_points.push_back(p2);
            point.bounding_points.push_back(p3);
            // save qr bounding box coordinates

            float distance = mx/box.size.width + 0.25; // px * m / px (detected width of qr box in image)
            
            int pixels_y = box.center.x - width/2; // center x image axis displacement in pixels
            int pixels_z = box.center.y - height/2; // center y image axis displacement in pixels
            float pixel_value_y = 0.168/(box.size.width); // how much distance a single pixel represent
            float pixel_value_z = 0.168/(box.size.height); 
            point.code = interpretation.message;
            if (angles.z < 0){
              angles.z = M_PI + angles.z + M_PI;
            }
            
            arma::fmat Qr_to_camera (4,1, arma::fill::ones); // Qr to camera position vector
            Qr_to_camera(0, 0) = distance;
            Qr_to_camera(1, 0) = float(pixels_y) * pixel_value_y * -1.0f;
            Qr_to_camera(2, 0) = float(pixels_z) * pixel_value_z * -1.0f;

            arma::fmat Drone_to_world (4,4, arma::fill::eye); // Drone to world transformation matrix
            Drone_to_world(0, 0) = cos(angles.z);
            Drone_to_world(0, 1) = -sin(angles.z);
            Drone_to_world(1, 0) = sin(angles.z);
            Drone_to_world(1, 1) = cos(angles.z);
            Drone_to_world(0, 3) = selfPose.pose.position.x;
            Drone_to_world(1, 3) = selfPose.pose.position.y;
            Drone_to_world(2, 3) = selfPose.pose.position.z;

            arma::fmat Qr_to_world = Drone_to_world * Qr_to_camera; // Qr to world result position vector

            point.point.x = Qr_to_world (0, 0);
            point.point.y = Qr_to_world (1, 0);
            point.point.z = Qr_to_world (2, 0);     
          
            list_points.list_of_qr_codes.push_back(point);
                            
          }
      }
      if(!entered){
        interpretation.message="";
        notification_pub.publish(interpretation);}
      
      else {
        qr_code_localized_pub.publish(list_points);
        std::cout << list_points.list_of_qr_codes.size() << std::endl;
        list_points.list_of_qr_codes.clear();
      }

      if (interpretation.message == ""){
        aerostack_msgs::QrCodeLocalized point;
        point.code = interpretation.message;
        point.point.x = 0;
        point.point.y = 0;
        point.point.z = 0;
        list_points.list_of_qr_codes.push_back(point);
        qr_code_localized_pub.publish(list_points);
        list_points.list_of_qr_codes.clear();
      }
    }
}

void QrFeatureExtractor::CameraInfoCallback (const sensor_msgs::CameraInfo &msg){         
    camera_info = msg;
    float length = 0.168; //horizontal length of qr image
    mx = camera_info.K[0] * length; // focal length (px) * horizontal length (m)
}


geometry_msgs::Vector3 QrFeatureExtractor::ToEulerAngles(geometry_msgs::Quaternion q) {

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.y = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.z = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}


void QrFeatureExtractor::ownStart(){
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);

    // enable qr
    scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
 
}

void QrFeatureExtractor::ownRun(){

}

void QrFeatureExtractor::ownStop(){

}


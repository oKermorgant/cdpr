#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

//How to include all necessary headers in one line?
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMeterPixelConversion.h>

//Define a list of necessary cable names and insert them into vector object. NOT NEEDED in C++11
//const char *cable_list[] = {"cube::cable0", "cube::cable1", "cube::cable2", "cube::cable3"};

class Extractor
{
public:
  const std::vector<std::string> target_cables;
  std::vector<int> cable_indices;
  const std::string target_cam;
  int cam_index;

  std::vector<geometry_msgs::Pose> cable_poses;
  geometry_msgs::Pose camera_pose;
  bool contains_poses;

  cv::Mat cam_image;
  bool contains_image;

public:
  /*Extractor(): target_cables(cable_list, cable_list + sizeof(cable_list) / sizeof(cable_list[0])),
    target_cam("cam1::camera_link"), contains_poses(false), contains_image(false) { } */
  Extractor(): target_cables{"cube::cable0", "cube::cable1", "cube::cable2", "cube::cable3"},
    target_cam("cam1::camera_link"), contains_poses(false), contains_image(false) { }
  void extractIndexCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
  void updatePoseCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
  void updateImageCallback(const sensor_msgs::Image::ConstPtr& msg);
  vpFeaturePoint cablePoint2FeaturePoint(int cableIndex, vpPoint P);
  vpImagePoint featurePoint2ImagePixel(vpFeaturePoint p);
};

//************* IndexExtractor initialization and definition BEGIN

void Extractor::extractIndexCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
  this->cable_indices.clear();
  //extract cable indices
  for (int i = 0; i < this->target_cables.size(); i++){
      for (int j = 0; j < msg->name.size(); j++){
          if (this->target_cables[i].compare(msg->name[j]) == 0){
              this->cable_indices.push_back(j);
            }
        }
    }

  //extract camera index
  for (int i = 0; i < msg->name.size(); i++){
      if (this->target_cam.compare(msg->name[i]) == 0){
          this->cam_index = i;
        }
    }

  //ROS_INFO("extractIndexCallback executed!");
}

void Extractor::updatePoseCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
  this->cable_poses.clear();
  for(int i = 0; i < this->cable_indices.size(); i++){
      this->cable_poses.push_back(msg->pose[this->cable_indices[i]]);
    }
  this->camera_pose = msg->pose[this->cam_index];

  this->contains_poses = true;

  //ROS_INFO("updatePoseCallback executed!");
}

void Extractor::updateImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  this->cam_image = cv_bridge::toCvShare(msg, "bgr8")->image;
  this->contains_image = true;

  //ROS_INFO("updateImageCallback executed!");
}

vpFeaturePoint Extractor::cablePoint2FeaturePoint(int cableIndex, vpPoint P)
{
  vpTranslationVector t_cable(this->cable_poses[cableIndex].position.x,
                              this->cable_poses[cableIndex].position.y,
                              this->cable_poses[cableIndex].position.z);
  vpQuaternionVector q_cable(this->cable_poses[cableIndex].orientation.x,
                             this->cable_poses[cableIndex].orientation.y,
                             this->cable_poses[cableIndex].orientation.z,
                             this->cable_poses[cableIndex].orientation.w);
  vpHomogeneousMatrix Mcable(t_cable,q_cable);

  vpTranslationVector t_cam(this->camera_pose.position.x,
                            this->camera_pose.position.y,
                            this->camera_pose.position.z);
  vpQuaternionVector q_cam(this->camera_pose.orientation.x,
                           this->camera_pose.orientation.y,
                           this->camera_pose.orientation.z,
                           this->camera_pose.orientation.w);
  vpHomogeneousMatrix Mcam(t_cam,q_cam);

  //define important transformation from standart camera model
  //to gazebo camera model
  std::vector<double> v(12, 0);
  v[1] = -1; v[6] = -1; v[8] = 1;
  vpHomogeneousMatrix M_stdCam_gazeboCam(v);

  vpHomogeneousMatrix M = M_stdCam_gazeboCam * Mcam.inverse() * Mcable;

  vpFeaturePoint p;
  P.track(M);
  vpFeatureBuilder::create (p, P);

  return p;
}

vpImagePoint Extractor::featurePoint2ImagePixel(vpFeaturePoint p)
{
  //camera parameters obtained from /cam1/camera/camera_info topic
  vpCameraParameters cam(554,554,320,240);

  vpImagePoint ip;
  vpMeterPixelConversion::convertPoint(cam, p.get_x(), p.get_y(), ip);

  return ip;
}

//************* IndexExtractor initialization and definition END

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pseudo_feature_extractor");
  ros::NodeHandle n("~");

  Extractor extractor;

  //Very important to set here queueSize = 1, as I want to process only a single message!
  int queue_size = 1;
  ros::Subscriber index_sub = n.subscribe<gazebo_msgs::LinkStates>("link_states", queue_size,
                                                                   &Extractor::extractIndexCallback, &extractor);

  //The following 4 lines are used to process a single message and shutdown the subscriber.
  //Probably NOT the best way!
  ros::Rate r1(100);
  while(index_sub.getNumPublishers() == 0) r1.sleep();
  ros::spinOnce();
  index_sub.shutdown();

  for(int i = 0; i < extractor.cable_indices.size(); i++){
      ROS_INFO("Cable index: [%d] %s", extractor.cable_indices[i], extractor.target_cables[i].c_str());
    }
  ROS_INFO("Camera index: [%d] %s", extractor.cam_index, extractor.target_cam.c_str());

  queue_size = 1;
  ros::Subscriber pose_sub = n.subscribe<gazebo_msgs::LinkStates>("link_states", queue_size,
                                                                  &Extractor::updatePoseCallback, &extractor);

  ros::Subscriber cam_sub = n.subscribe<sensor_msgs::Image>("image_in", queue_size,
                                                            &Extractor::updateImageCallback, &extractor);
  ros::Publisher feature_pub = n.advertise<std_msgs::Float64MultiArray>("features_out", queue_size);



  ros::Rate r2(30);
  while(ros::ok())
    {
      ros::spinOnce();

      if(extractor.contains_poses) {
          std::stringstream output;
          output << "\n---MESSAGE---\n";

          for(int i = 0; i < extractor.cable_indices.size(); i++){
              output << "Cable " << i << " Index: " << extractor.cable_indices[i] << "\n"
                     << "Name: " << extractor.target_cables[i] << "\n"
                     << "Position: " << extractor.cable_poses[i].position.x << " "
                     << extractor.cable_poses[i].position.y << " "
                     << extractor.cable_poses[i].position.z << "\n";
              output << "Orientation: " << extractor.cable_poses[i].orientation.x << " "
                     << extractor.cable_poses[i].orientation.y << " "
                     << extractor.cable_poses[i].orientation.z << " "
                     << extractor.cable_poses[i].orientation.w << "\n";
            }
          output << "Camera index: " << extractor.cam_index << "\n"
                 << "Name: " << extractor.target_cam << "\n"
                 << "Position: " << extractor.camera_pose.position.x << " "
                 << extractor.camera_pose.position.y << " "
                 << extractor.camera_pose.position.z << "\n"
                 << "Orientation: " << extractor.camera_pose.orientation.x << " "
                 << extractor.camera_pose.orientation.y << " "
                 << extractor.camera_pose.orientation.z << " "
                 << extractor.camera_pose.orientation.w << "\n";
          output << "\n";

          //ROS_INFO("%s", output.str().c_str());
        } else {
          //ROS_INFO("Poses not yet received!");
        }

      if(extractor.contains_poses && extractor.contains_image){

          std_msgs::Float64MultiArray feature_msg;

          for (int i = 0; i < extractor.cable_poses.size(); i++){
              vpPoint P1(0,0,1), P2(0,0,3);

              vpFeaturePoint p1 = extractor.cablePoint2FeaturePoint(i, P1);
              vpFeaturePoint p2 = extractor.cablePoint2FeaturePoint(i, P2);
              vpImagePoint ip1 = extractor.featurePoint2ImagePixel(p1);
              vpImagePoint ip2 = extractor.featurePoint2ImagePixel(p2);

              //ROS_INFO("j i: %f %f", ip2.get_j(), ip2.get_i());
              //ROS_INFO("u v: %f %f", ip2.get_u(), ip2.get_v());

              double theta = std::atan2(p2.get_y()-p1.get_y(), p2.get_x()-p1.get_x());
              //ROS_INFO("Cable: %d Theta in degrees: %f", i, theta*180/M_PI);

              feature_msg.data.push_back(theta);

              cv::line(const_cast<cv::Mat&>(extractor.cam_image),
                       cv::Point2f(ip1.get_j(),ip1.get_i()),
                       cv::Point2f(ip2.get_j(),ip2.get_i()),
                       cv::Scalar(0, 250, 250), 2);
              cv::circle(const_cast<cv::Mat&>(extractor.cam_image),
                         cv::Point2f(ip2.get_j(),ip2.get_i()),
                         5, cv::Scalar(250, 0, 0), 3);
            }

          feature_pub.publish(feature_msg);

          cv::imshow("view", extractor.cam_image);
          cv::waitKey(30);

        } else {

          ROS_INFO("Poses or Image not yet received!");

        }

      r2.sleep();
    }

  return 0;
}

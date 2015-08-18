#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#define CONTROLLER
#define dDOUBLE
#define USE_ODE
//SIGverse
#include "SimObjController.hpp"
#include <sig_ros/SetWheel.h>
#include <sig_ros/SetWheelVelocity.h>
#include <sig_ros/ik.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <rosgraph_msgs/Clock.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// PI
#include <boost/math/constants/constants.hpp>

#include <moveit_msgs/GetPositionIK.h>
#include <math.h>
#include <string>
#include <ctype.h>
#include <stdio.h>
#include "matrix.hpp"
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

class RobotController : public SimObjController
{
   public:
      void onInit(InitEvent &evt);
      double onAction(ActionEvent &evt);

   private:
      void calculateTransform(ros::Time time);
      void scan(ros::Time time);
      void fillPositionArms(std::string arm);
      bool ik(sig_ros::ik::Request &req, sig_ros::ik::Response &res); 
      bool getJointPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res);
      float norm(Vector3d v1, Vector3d v2);
      Vector3d pointAfterRotation(Vector3d point, Vector3d center, float angle);
      std::string getLetter(std::string arm);
      std::string toString(double x);
      std::string getJointLength(std::string letter, Vector3d& torso_lift);
      //Robot
      void setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel);
      void setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel);

      RobotObj *myRobot;

      Vector3d position;
      std::map<std::string, Vector3d> positionArms;
      Rotation rotation;
      ViewService *m_view;

      //Topic
      ros::Subscriber setWheel_sub;
      ros::Subscriber setWheelVelocity_sub;
      ros::Publisher scan_pub;
      ros::Publisher clock_pub;

      ros::ServiceServer serviceGetJointPosition;
      ros::ServiceServer serviceIK;      
      //const Eigen::Affine3d end_effector_default_pose;
      robot_model::RobotModelPtr kinematic_model;
//      const robot_model::JointModelGroup* joint_model_group;
      std::map<std::string, const robot_model::JointModelGroup*> joint_model_group;
      //Robot
      double m_radius;           // radius of the wheel
      double m_distance;

      ros::Time current_time, last_time;

      //rosbag::View view;

      std::vector<std::string> topics;
      rosbag::Bag bag;
};  

#endif

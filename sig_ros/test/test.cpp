// Bring in my package's API, which is what I'm testing
//#include <sig_ros/sig_controller.hpp>
#include <ros/ros.h>
#include <sig_ros/getObjPosition.h>
#include <sig_ros/getPartsPosition.h>
#include <sig_ros/getRotation.h>
#include <sig_ros/getAngleRotation.h>
#include <sig_ros/getJointAngle.h>
#include <sstream>
// Bring in gtest
#include <gtest/gtest.h>
//Run all tests : catkin_make run_tests

// Declare a test
TEST(TestSuite, testCaseGetObjPosition)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetObjPosition = n.serviceClient<sig_ros::getObjPosition>("robot_000_get_obj_position");
   sig_ros::getObjPosition srvGetObjPosition;
   srvGetObjPosition.request.name = "robot_000";
	if (serviceGetObjPosition.call(srvGetObjPosition)) {
	   EXPECT_EQ(83.36,  srvGetObjPosition.response.posX);
	   EXPECT_EQ(81.6,  srvGetObjPosition.response.posY);
	   EXPECT_EQ(16,  srvGetObjPosition.response.posZ);
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_obj_position");
	}
}

TEST(TestSuite, testCaseGetPartsPosition)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetPartsPosition = n.serviceClient<sig_ros::getPartsPosition>("robot_000_get_parts_position");
   sig_ros::getPartsPosition srvGetPartsPosition;
   srvGetPartsPosition.request.part = "RARM_LINK2";
	if (serviceGetPartsPosition.call(srvGetPartsPosition)) {
	   EXPECT_EQ(83.36,  srvGetPartsPosition.response.posX);
	   EXPECT_EQ(81.6,  srvGetPartsPosition.response.posY);
	   EXPECT_EQ(16,  srvGetPartsPosition.response.posZ);
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_parts_position");
	}
}

TEST(TestSuite, testCaseGetRotation)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetRotation = n.serviceClient<sig_ros::getRotation>("robot_000_get_rotation");
   sig_ros::getRotation srvGetRotation;
   srvGetRotation.request.axis = "y";
	if (serviceGetRotation.call(srvGetRotation)) {
	   EXPECT_EQ(0,  srvGetRotation.response.qY);
	   EXPECT_EQ(1,  srvGetRotation.response.qW);
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_rotation");
	}
}

TEST(TestSuite, testCaseGetAngleRotation)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetAngleRotation = n.serviceClient<sig_ros::getAngleRotation>("robot_000_get_rotation");
   sig_ros::getAngleRotation srvGetAngleRotation;
   srvGetAngleRotation.request.axis = "z";
	if (serviceGetAngleRotation.call(srvGetAngleRotation)) {
	   EXPECT_EQ(0,  srvGetAngleRotation.response.angle);
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_angle_rotation");
	}
}

TEST(TestSuite, testCaseGetJointAngleRARM_JOINT1)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetJointAngle = n.serviceClient<sig_ros::getJointAngle>("robot_000_get_joint_angle");
   sig_ros::getJointAngle srvGetJointAngle;
   srvGetJointAngle.request.nameArm = "RARM_JOINT1";
	if (serviceGetJointAngle.call(srvGetJointAngle)) {
	   EXPECT_EQ(0,  srvGetJointAngle.response.angle);
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_joint_angle");
	}
}

TEST(TestSuite, testCaseGetJointAngleRARM_JOINT4)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetJointAngle = n.serviceClient<sig_ros::getJointAngle>("robot_000_get_joint_angle");
   sig_ros::getJointAngle srvGetJointAngle;
   srvGetJointAngle.request.nameArm = "RARM_JOINT4";
	if (serviceGetJointAngle.call(srvGetJointAngle)) {
	   EXPECT_EQ(0,  srvGetJointAngle.response.angle);
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_joint_angle");
	}
}
// Declare another test
/*TEST(TestSuite, testCase2)
{
<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "gira");
  return RUN_ALL_TESTS();
}

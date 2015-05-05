// Bring in my package's API, which is what I'm testing
//#include <sig_ros/sig_controller.hpp>
#include <ros/ros.h>
#include <sig_ros/getObjPosition.h>
#include <sig_ros/getPartsPosition.h>
#include <sig_ros/getRotation.h>
#include <sig_ros/getAngleRotation.h>
#include <sig_ros/getJointAngle.h>
#include <sig_ros/checkService.h>
#include <sig_ros/connectToService.h>
#include <sig_ros/getAngleRotation.h>
#include <sig_ros/getCollisionStateOfMainPart.h>
#include <sig_ros/getEntities.h>
#include <sstream>
// Bring in gtest
#include <gtest/gtest.h>
//Run all tests : catkin_make run_tests

// Declare a test
TEST(TestSuiteRobotController, testCaseGetObjPosition)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetObjPosition = n.serviceClient<sig_ros::getObjPosition>("robot_000_get_obj_position");
   sig_ros::getObjPosition srvGetObjPosition;
   srvGetObjPosition.request.name = "robot_000";
	if (serviceGetObjPosition.call(srvGetObjPosition)) {
	   EXPECT_EQ(100,  srvGetObjPosition.response.posX);
	   EXPECT_EQ(30,  srvGetObjPosition.response.posY);
	   EXPECT_EQ(10,  srvGetObjPosition.response.posZ);
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_obj_position");
	}
}

TEST(TestSuiteRobotController, testCaseGetPartsPosition)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetPartsPosition = n.serviceClient<sig_ros::getPartsPosition>("robot_000_get_parts_position");
   sig_ros::getPartsPosition srvGetPartsPosition;
   srvGetPartsPosition.request.part = "RARM_LINK2";
	if (serviceGetPartsPosition.call(srvGetPartsPosition)) {
	   EXPECT_FLOAT_EQ(83.36,  srvGetPartsPosition.response.posX);
	   EXPECT_EQ(81.6,  srvGetPartsPosition.response.posY);
	   EXPECT_EQ(16,  srvGetPartsPosition.response.posZ);
	}  else {
	   ROS_ERROR("Failed to call service robot_000_get_parts_position");
	}
}

TEST(TestSuiteRobotController, testCaseGetRotation)
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

TEST(TestSuiteRobotController, testCaseGetAngleRotation)
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

TEST(TestSuiteRobotController, testCaseGetJointAngleRARM_JOINT1)
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

TEST(TestSuiteRobotController, testCaseGetJointAngleRARM_JOINT4)
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

/*******************Test SimObjController*************************/
// Declare another test
TEST(TestSuiteSimObjController, testCaseCheckService)
{
   ros::NodeHandle n;
   ros::ServiceClient service = n.serviceClient<sig_ros::checkService>("trashbox_0_check_service");
   sig_ros::checkService srv;
   srv.request.serviceName = "RobocupReferee";
	if (service.call(srv)) {
	   EXPECT_FALSE(srv.response.connected);
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_check_service");
	}
}

TEST(TestSuiteSimObjController, testCaseConnectToService)
{
   ros::NodeHandle n;
   ros::ServiceClient service = n.serviceClient<sig_ros::connectToService>("trashbox_0_connect_to_service");
   sig_ros::connectToService srv;
   srv.request.serviceName = "RobocupReferee";
	if (service.call(srv)) {
	   EXPECT_FALSE(srv.response.connected);
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_connect_to_service");
	}
}

TEST(TestSuiteSimObjController, testCaseGetAngleRotation)
{
   ros::NodeHandle n;
   ros::ServiceClient service = n.serviceClient<sig_ros::getAngleRotation>("trashbox_0_get_angle_rotation");
   sig_ros::getAngleRotation srv;
   srv.request.axis = "x";
   srv.request.x = 10;
   srv.request.y = 90;
   srv.request.z = 0;
	if (service.call(srv)) {
	   EXPECT_FLOAT_EQ(0.11043153, srv.response.angle);
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_get_angle_rotation x");
	}
	srv.request.axis = "y";
	if (service.call(srv)) {
	   EXPECT_FLOAT_EQ(0.99388373, srv.response.angle);
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_get_angle_rotation y");
	}
	srv.request.axis = "z";
	if (service.call(srv)) {
	   EXPECT_EQ(0, srv.response.angle);
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_get_angle_rotation z");
	}
}

TEST(TestSuiteSimObjController, testGetCollisionOfMainPart)
{
   ros::NodeHandle n;
   ros::ServiceClient service = n.serviceClient<sig_ros::getCollisionStateOfMainPart>("trashbox_0_get_collision_state_of_main_part");
   sig_ros::getCollisionStateOfMainPart srv;
	if (service.call(srv)) {
	   EXPECT_FALSE(srv.response.collisionState);
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_get_collision_state_of_main_part");
	}
}

TEST(TestSuiteSimObjController, testGetEntities)
{
   ros::NodeHandle n;
   ros::ServiceClient service = n.serviceClient<sig_ros::getEntities>("trashbox_0_get_entities");
   sig_ros::getEntities srv;
	if (service.call(srv)) {
	   EXPECT_EQ(26, srv.response.length);//TODO test string[] entitiesNames
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_get_entities");
	}
}

TEST(TestSuiteSimObjController, testGetJointAngle)
{
   ros::NodeHandle n;
   ros::ServiceClient service = n.serviceClient<sig_ros::getJointAngle>("trashbox_0_get_joint_angle");
   sig_ros::getJointAngle srv;
   //srv.request.nameArm = "RARM_JOINT1";
	if (service.call(srv)) {
	   EXPECT_EQ(0, srv.response.angle);
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_get_joint_angle");
	}
}

TEST(TestSuiteSimObjController, testCaseGetObjPosition)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetObjPosition = n.serviceClient<sig_ros::getObjPosition>("trashbox_0_get_obj_position");
   sig_ros::getObjPosition srvGetObjPosition;
   srvGetObjPosition.request.name = "trashbox_0";
	if (serviceGetObjPosition.call(srvGetObjPosition)) {
	   EXPECT_EQ(-100,  srvGetObjPosition.response.posX);
	   EXPECT_FLOAT_EQ(36.35,  srvGetObjPosition.response.posY);
	   EXPECT_EQ(-220,  srvGetObjPosition.response.posZ);
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_get_obj_position");
	}
}

TEST(TestSuiteSimObjController, testCaseGetRotation)
{
   ros::NodeHandle n;
   ros::ServiceClient serviceGetRotation = n.serviceClient<sig_ros::getRotation>("trashbox_0_get_rotation");
   sig_ros::getRotation srvGetRotation;
   srvGetRotation.request.axis = "y";
	if (serviceGetRotation.call(srvGetRotation)) {
	   EXPECT_EQ(0,  srvGetRotation.response.qY);
	   EXPECT_EQ(1,  srvGetRotation.response.qW);
	}  else {
	   ROS_ERROR("Failed to call service trashbox_0_get_rotation");
	}
}
/*****************************************************************/
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestController");
  return RUN_ALL_TESTS();
}

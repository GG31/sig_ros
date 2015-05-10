#include "include/RobotController.hpp"

 
void RobotController::onInit(InitEvent &evt)
{
   int argc = 0;
   char** argv = NULL;
   myRobot = getRobotObj(myname());
   ros::init(argc, argv, std::string(this->myname()) + "_sig_controller_node");//+std::string(this->myname())
   ros::NodeHandle n;
   //*n = nh;
   
   //Topics
   onRecvMsg_pub = n.advertise<sig_ros::MsgRecv>(std::string(this->myname())+"_onRecvMsg", 1000);
   onCollision_pub = n.advertise<sig_ros::OnCollision>(std::string(this->myname())+"_onCollisionMsg", 1000);
   setWheel_sub = n.subscribe<sig_ros::SetWheel>(std::string(this->myname()) + "_setWheel", 1, &RobotController::setWheelCallback, this);
   setWheelVelocity_sub = n.subscribe<sig_ros::SetWheelVelocity>(std::string(this->myname()) + "_setWheelVelocity", 1, &RobotController::setWheelVelocityCallback, this);
   setJointVelocity_sub = n.subscribe<sig_ros::SetJointVelocity>(std::string(this->myname()) + "_setJointVelocity", 1, &RobotController::setJointVelocityCallback, this);
   releaseObj_sub = n.subscribe<sig_ros::ReleaseObj>(std::string(this->myname()) + "_releaseObj", 1, &RobotController::releaseObjCallback, this);

   //Srv
   service = n.advertiseService(std::string(this->myname()) + "_get_time", &RobotController::getTime, this);
   serviceGetObjPosition = n.advertiseService(std::string(this->myname()) + "_get_obj_position", &RobotController::getObjPosition, this);
   serviceGetPartsPosition = n.advertiseService(std::string(this->myname()) + "_get_parts_position", &RobotController::getPartsPosition, this);
   serviceGetRotation = n.advertiseService(std::string(this->myname()) + "_get_rotation", &RobotController::getRotation, this);
   serviceGetAngleRotation = n.advertiseService(std::string(this->myname()) + "_get_angle_rotation", &RobotController::getAngleRotation, this);
   serviceGetJointAngle = n.advertiseService(std::string(this->myname()) + "_get_joint_angle", &RobotController::getJointAngle, this);
   serviceGraspObj = n.advertiseService(std::string(this->myname()) + "_grasp_obj", &RobotController::graspObj, this);
 
   m_simulatorTime = 0;
}

double RobotController::onAction(ActionEvent &evt)
{
   ros::spinOnce();
   m_simulatorTime = evt.time();
   return 0.01;
}

void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
   sig_ros::MsgRecv msg;
   msg.sender = evt.getSender();
   msg.content = evt.getMsg();
   onRecvMsg_pub.publish(msg);
}

void RobotController::onCollision(CollisionEvent &evt)
{
	// Get name of entity which is touched by the robot
	const std::vector<std::string> & with = evt.getWith();
	// Get parts of the robot which is touched by the entity
	const std::vector<std::string> & mparts = evt.getMyParts();
	sig_ros::OnCollision msg;
	for (int i=0; i < with.size(); i++) {
      msg.name = with[i];
      msg.part = mparts[i];
      onCollision_pub.publish(msg);
   }
}



/*****************************Callback topic************************/

void RobotController::setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel)
{
   std::cout << "setWheelCallback" << std::endl;
   m_radius = wheel->wheelRadius;           // radius of the wheel
	m_distance = wheel->wheelDistance; 
   myRobot->setWheel(m_radius, m_distance);
}

void RobotController::setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel)
{
   std::cout << "setWheelVelocityCallback" << std::endl;
   myRobot->setWheelVelocity(wheel->leftWheel, wheel->rightWheel);
}

void RobotController::setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg)
{
   myRobot->setJointVelocity(msg->jointName.c_str(), msg->angularVelocity, msg->max);
}

void RobotController::releaseObjCallback(const sig_ros::ReleaseObj::ConstPtr& msg) {
   CParts *parts = myRobot->getParts(msg->arm.c_str());
	// release grasping
	parts->releaseObj();
}

/*****************************End callback topic************************/


/*******************************Srv***********************************/
bool RobotController::getTime(sig_ros::getTime::Request &req, sig_ros::getTime::Response &res)
{
   res.time = m_simulatorTime;
   return true;
}

bool RobotController::getObjPosition(sig_ros::getObjPosition::Request &req, sig_ros::getObjPosition::Response &res)
{
   SimObj *obj = getObj(req.name.c_str());
   Vector3d pos;
   obj->getPosition(pos);
   res.posX = pos.x();
   res.posY = pos.y();
   res.posZ = pos.z();
   return true;
}

bool RobotController::getPartsPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res)
{
   Vector3d pos;
   if (req.name == "") {
      myRobot->getPartsPosition (pos, req.part.c_str());
   } else {
      SimObj *obj = getObj(req.name.c_str());
      if (obj == NULL) {
         return false;
      }
      obj->getPartsPosition(pos, req.part.c_str());
   }
   res.posX = pos.x();
   res.posY = pos.y();
   res.posZ = pos.z();
   return true;
}

bool RobotController::getRotation(sig_ros::getRotation::Request &req, sig_ros::getRotation::Response &res)
{
   Rotation ownRotation;
   if (req.name == "") {
	   myRobot->getRotation(ownRotation);
	} else {
	   SimObj *obj = getObj(req.name.c_str());
	   if (obj == NULL) {
         return false;
      }
      obj->getRotation(ownRotation);
	}
   res.qW = ownRotation.qw();
   res.qY = ownRotation.qy();
   res.qX = ownRotation.qx();
   res.qZ = ownRotation.qz();
   return true;
}

bool RobotController::getAngleRotation(sig_ros::getAngleRotation::Request &req, sig_ros::getAngleRotation::Response &res)
{
   Rotation ownRotation;
	myRobot->getRotation(ownRotation);
	Vector3d vector3d = Vector3d(req.x, req.y, req.z);
	if (req.axis == "z") { 
	   res.angle = vector3d.angle(Vector3d(0.0, 0.0, 1.0));
	} else {
	   std::cout << "Not z : " << req.axis.c_str() << std::endl;
	}
   return true;
}

bool RobotController::getJointAngle(sig_ros::getJointAngle::Request &req, sig_ros::getJointAngle::Response &res)
{
   res.angle = myRobot->getJointAngle(req.nameArm.c_str());
   return true;
}

bool RobotController::graspObj(sig_ros::graspObj::Request &req, sig_ros::graspObj::Response &res)
{
   CParts *parts = myRobot->getParts(req.part.c_str());
   res.ok = parts->graspObj(req.obj.c_str());
   return true;
}
/*****************************End Srv*********************************/

extern "C"  RobotController * createController ()
{
   return new RobotController;
}

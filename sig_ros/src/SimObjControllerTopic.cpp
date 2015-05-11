#include "include/SimObjController.hpp"

/*****************************Callback topic************************/
void SimObjController::setJointVelocityCallback(const sig_ros::SetJointVelocity::ConstPtr& msg)
{
   my->setJointVelocity(msg->jointName.c_str(), msg->angularVelocity, msg->max);
}

void SimObjController::releaseObjCallback(const sig_ros::ReleaseObj::ConstPtr& msg) {
   CParts *parts = my->getParts(msg->arm.c_str());
	// release grasping
	parts->releaseObj();
}

void SimObjController::setAxisAndAngleCallback(const sig_ros::SetAxisAndAngle::ConstPtr& msg) {
   if (msg->name == "") {
      my->setAxisAndAngle(msg->axisX, msg->axisY, msg->axisZ, msg->angle);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setAxisAndAngle(msg->axisX, msg->axisY, msg->axisZ, msg->angle);
   }
}

void SimObjController::setPositionCallback(const sig_ros::SetPosition::ConstPtr& msg) {
   if (msg->name == "") {
      my->setPosition(msg->posX, msg->posY, msg->posZ);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setPosition(msg->posX, msg->posY, msg->posZ);
   }
}

void SimObjController::setAccelCallback(const sig_ros::Double3D::ConstPtr& msg) {
   if (msg->name == "") {
      my->setAccel(msg->x, msg->y, msg->z);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setAccel(msg->x, msg->y, msg->z);
   }
}

void SimObjController::setAngularVelocityCallback(const sig_ros::Double3D::ConstPtr& msg) {
   if (msg->name == "") {
      my->setAngularVelocity(msg->x, msg->y, msg->z);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setAngularVelocity(msg->x, msg->y, msg->z);
   }
}

void SimObjController::setTorqueCallback(const sig_ros::Double3D::ConstPtr& msg) {
   if (msg->name == "") {
      my->setTorque(msg->x, msg->y, msg->z);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setTorque(msg->x, msg->y, msg->z);
   }
}

void SimObjController::setVelocityCallback(const sig_ros::Double3D::ConstPtr& msg) {
   if (msg->name == "") {
      my->setLinearVelocity(msg->x, msg->y, msg->z);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setLinearVelocity(msg->x, msg->y, msg->z);
   }
}

void SimObjController::setCollisionEnableCallback(const sig_ros::SetCollisionEnable::ConstPtr& msg) {
   if (msg->name == "") {
      my->setCollisionEnable(msg->flag);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setCollisionEnable(msg->flag);
   }
}
void SimObjController::setGravityModeCallback(const sig_ros::SetGravityMode::ConstPtr& msg) {
   if (msg->name == "") {
      my->setGravityMode(msg->gravity);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setGravityMode(msg->gravity);
   }
}
void SimObjController::setJointAngleCallback(const sig_ros::SetJointAngle::ConstPtr& msg) {
   if (msg->name == "") {
      my->setJointAngle(msg->jointName.c_str(), msg->angle);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setJointAngle(msg->jointName.c_str(), msg->angle);
   }
}
void SimObjController::setJointQuaternionCallback(const sig_ros::SetJointQuaternion::ConstPtr& msg) {
   if (msg->name == "") {
      my->setJointQuaternion(msg->jointName.c_str(), msg->qW, msg->qX, msg->qY, msg->qZ, msg->offset); //TODO ajouter offset default
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setJointQuaternion(msg->jointName.c_str(), msg->qW, msg->qX, msg->qY, msg->qZ, msg->offset);
   }
}
void SimObjController::setMassCallback(const sig_ros::SetMass::ConstPtr& msg) {
   if (msg->name == "") {
      my->setMass(msg->mass);
   } else {
      SimObj *obj = getObj(msg->name.c_str());
      obj->setMass(msg->mass);
   }
}
/*****************************End callback topic************************/

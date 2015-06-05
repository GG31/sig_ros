/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include "include/RobotCommand.hpp"

RobotCommand::RobotCommand() {
   init();
}

void RobotCommand::init() {
   m_radius = 10.0;
	m_distance = 10.0;
	//Topics
	robot_000_setWheel_pub = n.advertise<sig_ros::SetWheel>("robot_000_setWheel", 1000);
	robot_000_setWheelVelocity_pub = n.advertise<sig_ros::SetWheelVelocity>("robot_000_setWheelVelocity", 1000);
	robot_000_setJointVelocity_pub = n.advertise<sig_ros::SetJointVelocity>("robot_000_setJointVelocity", 1000);
	robot_000_releaseObj_pub = n.advertise<sig_ros::ReleaseObj>("robot_000_releaseObj", 1000);
	robot_000_onRecvMsg_sub = n.subscribe<sig_ros::MsgRecv>("robot_000_onRecvMsg", 1, &RobotCommand::onMsgRecvCallback, this);
	robot_000_onCollision_sub = n.subscribe<sig_ros::OnCollision>("robot_000_onCollisionMsg", 1, &RobotCommand::onCollisionCallback, this);
	//Srv
	serviceGetTime = n.serviceClient<sig_ros::getTime>("robot_000_get_time");
	serviceGetObjPosition = n.serviceClient<sig_ros::getObjPosition>("robot_000_get_obj_position");
	serviceGetPartsPosition = n.serviceClient<sig_ros::getPartsPosition>("robot_000_get_parts_position");
	serviceGetRotation = n.serviceClient<sig_ros::getRotation>("robot_000_get_rotation");
	serviceGetAngleRotation = n.serviceClient<sig_ros::getAngleRotation>("robot_000_get_angle_rotation");
	serviceGetJointAngle = n.serviceClient<sig_ros::getJointAngle>("robot_000_get_joint_angle");
   serviceGraspObj = n.serviceClient<sig_ros::graspObj>("robot_000_grasp_obj");
	
	ros::Duration(0.5).sleep(); //Because we have to wait that the publisher be ready
	msgSetWheel.wheelRadius = m_radius;
	msgSetWheel.wheelDistance = m_distance;
	robot_000_setWheel_pub.publish(msgSetWheel);
	ros::spinOnce();
	
	m_time = 0.0;
	m_time1 = 0.0;
	m_time4 = 0.0;

	m_state = 10;  // switch of initial behavior
	refreshRateOnAction = 0.1;     // refresh-rate for onAction proc.

	// angular velocity of wheel and moving speed of robot
	m_angularVelocity = 1.5;
	m_movingSpeed = m_angularVelocity*m_radius;  // conversion: rad/ms -> m/ms)

	// rotation speed of joint
	m_jointVelocity = 0.5;

	m_trashName1 = "petbottle_1";
	m_trashName2 = "can_0";

	m_trashBoxName1 = "trashbox_0";  // for recycle
	m_trashBoxName2 = "trashbox_1";  // for burnable

	m_grasp = false;
	
	
	// set positions;
   array_init(m_frontTrashBox1, -80.0, 0.0, -90);
   array_init(m_frontTrashBox2, 20.0, 0.0, -90);
   array_init(m_relayPoint1, 190.0, 0.0, -65.0);
   array_init(m_frontTrash1, 273.0, 0.0, -65.0);
   array_init(m_frontTrash2, 305.0, 0.0, -80.0);
   
}

void RobotCommand::array_init(double tab[], double v1, double v2, double v3) {
   tab[0] = v1;
   tab[1] = v2;
   tab[2] = v3;
}

void RobotCommand::onMsgRecvCallback(const sig_ros::MsgRecv::ConstPtr& msg)
{
   std::cout << msg->sender.c_str() << std::endl;
   std::cout << msg->content.c_str() << std::endl;
}

void RobotCommand::onCollisionCallback(const sig_ros::OnCollision::ConstPtr& msg)
{
   if (!m_grasp && (msg->name == m_trashName1 || msg->name == m_trashName2) && msg->part == "RARM_LINK7") {
      srvGraspObj.request.obj = msg->name;
      srvGraspObj.request.part = msg->part;
      if (serviceGraspObj.call(srvGraspObj)) {
         if (srvGraspObj.response.ok) {
            m_grasp = true; 
         }
      } else {
         ROS_ERROR("Failed to call service robot_000_grasp_obj");
      }
   }
}

void RobotCommand::stopRobotMove(void) {
	this->setWheelVelocity(0.0, 0.0);
}

double RobotCommand::loop(void) {
   double time = getTime();
   switch(m_state) {
		case 0: {
			break;
		}
		case 1: {
			this->stopRobotMove();
			break;
		}
		case 10: {  // go straight a bit
		   m_graspObjectName = m_trashName2;  // at first, focusing to m_trashName2:can_0
		   this->setWheelVelocity(m_angularVelocity, m_angularVelocity);

         m_time = 10.0/m_movingSpeed + time;
			m_state = 20;
			break;
		}
		case 20: {  // direct to the trash
	      if(time >= m_time && m_state == 20) {
			   this->stopRobotMove();    // at first, stop robot maneuver

			   double l_tpos[3];
			   this->recognizeObjectPosition(l_tpos, m_trashName2);
		      double l_moveTime = rotateTowardObj(l_tpos);  // rotate toward the position and calculate the time to be elapsed.

		      m_time = l_moveTime + time; //l_moveTime + time;
			   m_state = 30;
		   }
			break;
		}
		case 30: {  // proceed toward trash
			if(time >= m_time && m_state == 30) {
				this->stopRobotMove();

				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = goToObj(l_tpos, 75.0);  // go toward the position and calculate the time to be elapsed.

				m_time = l_moveTime + time;
				m_state = 40;
         }
         break;
    }
		case 40: {  // get back a bit after colliding with the table
			if(time >= m_time && m_state == 40) {
				this->stopRobotMove();    // at first, stop robot maneuver
            this->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 20./m_movingSpeed + time;
				
			}
         m_state = 50;
			break;
		}
		case 50: {  // detour: rotate toward relay point 1
			if(time >= m_time && m_state == 50) {
				this->stopRobotMove();

				double l_moveTime = rotateTowardObj(m_relayPoint1);

				m_time = l_moveTime + time;
				m_state = 60;
			}
			break;
		}
		case 60: {  // detour: go toward relay point 1
			if(time >= m_time && m_state == 60) {
				this->stopRobotMove();

				double l_moveTime = goToObj(m_relayPoint1, 0.0);

				m_time = l_moveTime + time;
				m_state = 70;
			}
			break;
		}
		case 70: {  // rotate toward the position in front of trash
			if(time >= m_time && m_state == 70) {
				this->stopRobotMove();

				double l_moveTime = rotateTowardObj(m_frontTrash1);

				m_time = l_moveTime + time;
				m_state = 80;
			}
			break;
		}
		case 80: {  // go toward the position in front of trash
			if(time >= m_time && m_state == 80) {
				this->stopRobotMove();

				double l_moveTime = goToObj(m_frontTrash1, 0.0);

				m_time = l_moveTime + time;
				m_state = 90;
			}
			break;
		}
		case 90: {  // rotate toward the trash
			if(time >= m_time && m_state == 90) {
				this->stopRobotMove();

				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = rotateTowardObj(l_tpos);

				m_time = l_moveTime + time;
				m_state = 100;
			}
			break;
		}
		case 100: {  // prepare the robot arm to grasping the trash
			if(time >= m_time && m_state == 100) {
				this->stopRobotMove();
				this->neutralizeArms(time);

				m_state = 105;
			}
			break;
		}
		case 105: {  // fix robot direction for grasping
			if(time >= m_time1 + 2 && m_state == 105) this->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(time >= m_time4 + 2 && m_state == 105) this->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(time >= m_time1 + 2 && time >= m_time4 + 2 && m_state == 105) {
				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = rotateTowardObj(l_tpos);

				m_time = l_moveTime + time;

				m_state = 110;
			}
			break;
		}
		case 110: {  // approach to the trash
			if(time >= m_time && m_state == 110) {
				this->stopRobotMove();


				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashName2);

				double l_moveTime = goToObj(l_tpos, 30.0);
				m_time = l_moveTime + time;

				m_state = 120;
			}
			break;
		}
		case 120: {  // try to grasp trash
			if(time >= m_time && m_state == 120) {
				this->stopRobotMove();
				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = goGraspingObject(l_tpos);
				m_time = l_moveTime + time;

				m_state = 125;
			}
			break;
		}
		case 125: {
			if(time >= m_time && m_state == 125) {
				this->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				this->neutralizeArms(time);

				m_state = 130;
			}
			break;
		}
		case 130: {
			if(time >= m_time1 && m_state == 130) this->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(time >= m_time4 && m_state == 130) this->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(time >= m_time1 && time >= m_time4 && m_state == 130) {

				this->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 20./m_movingSpeed + time;

				m_state = 150;
			}
			break;
		}
		case 150: {
			if(time >= m_time && m_state == 150) {
				this->stopRobotMove();
				double l_moveTime = rotateTowardObj(m_frontTrashBox2);

				m_time = l_moveTime + time;
				m_state = 160;
			}
			break;
		}
		case 160: {
			if(time >= m_time && m_state == 160) {
				this->stopRobotMove();
				double l_moveTime = goToObj(m_frontTrashBox2,0.0);
				m_time = l_moveTime + time;
				m_state = 161;
			}
			break;
		}
		case 161: {
			if(time >= m_time && m_state == 161) {
				this->stopRobotMove();
				this->prepareThrowing(time);

				m_state = 165;
			}
			break;
		}
		case 165: {
			if(time >= m_time1 && m_state == 165) this->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(time >= m_time4 && m_state == 165) this->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(time >= m_time1 && time >= m_time4 && m_state == 165) {

				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashBoxName2);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + time;

				m_state = 170;
			}
			break;
		}
		case 170: {
			if(time >= m_time && m_state == 170) {

				this->stopRobotMove();
				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashBoxName2);
				double l_moveTime = goToObj(l_tpos, 50.0);
				m_time = l_moveTime + time;

				m_state = 180;
			}
			break;
		}
		case 180: {
			if(time >= m_time && m_state == 180) {
				this->stopRobotMove();
				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashBoxName2);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + time;

				m_state = 200;
			}
			break;
		}
		case 200: {  // throw trash and get back a bit
			if(time >= m_time && m_state == 200) {

				this->stopRobotMove();
				this->throwTrash();

				sleep(1);

				this->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 50.0/m_movingSpeed + time;

				m_state = 225;
			}
			break;
		}
		case 225: {  // recover robot arms
			if(time >= m_time && m_state == 225) {
				this->stopRobotMove();
				this->neutralizeArms(time);

				m_state = 240;
			}
			break;
		}
//********************************************************************
		case 240: {  // go next
			if(time >= m_time1 && m_state == 240) this->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(time >= m_time4 && m_state == 240) this->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(time >= m_time1 && time >= m_time4 && m_state == 240) {
				this->stopRobotMove();
				m_graspObjectName = m_trashName1;  // set next target
				double l_moveTime = rotateTowardObj(m_frontTrash2);
				m_time = l_moveTime + time;

				m_state = 250;
			}
			break;
		}
		case 250: {  // approach to neighbor of next target
			if(time >= m_time && m_state == 250) {
				this->stopRobotMove();
				double l_moveTime = goToObj(m_frontTrash2, 0.0);
				m_time = l_moveTime + time;

				m_state = 260;
			}
			break;
		}
		case 260: {
			if(time >= m_time && m_state == 260) {
				this->stopRobotMove();
				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashName1);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + time;

				m_state = 270;
			}
			break;
		}
		case 270: {  // approach to next target
			if(time >= m_time && m_state == 270) {
				this->stopRobotMove();

				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashName1);
				double l_moveTime = goToObj(l_tpos, 39.0);
				m_time = l_moveTime + time;

				m_state = 275;
			}
			break;
		}
		case 275: {
			if(time >= m_time && m_state == 275) {
				this->stopRobotMove();
				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashName1);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + time;

				m_state = 280;
			}
			break;
		}
		case 280: {
			if(time >= m_time && m_state == 280) {
				this->stopRobotMove();

				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashName1);
				double l_moveTime = goGraspingObject(l_tpos);

				m_time = l_moveTime + time;

				m_state = 290;
			}
			break;
		}
		case 290: {
			if(time >= m_time && m_state == 290) {
				this->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				this->neutralizeArms(time);

				m_state = 300;
			}
			break;
		}
		case 300: {
			if(time >= m_time1 && m_state == 300) this->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(time >= m_time4 && m_state == 300) this->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(time >= m_time1 && time >= m_time4 && m_state == 300) {

				this->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 20./m_movingSpeed + time;

				m_state = 310;
			}
			break;
		}
		case 310: {
			if(time >= m_time && m_state == 310) {
				this->stopRobotMove();
				double l_moveTime = rotateTowardObj(m_frontTrashBox1);
				m_time = l_moveTime + time;

				m_state = 320;
			}
			break;
		}

		case 320: {
			if (time >= m_time && m_state == 320) {
				this->stopRobotMove();
				double l_moveTime = goToObj(m_frontTrashBox1,0.0);
				m_time = l_moveTime + time;

				m_state = 340;
			}
			break;
		}
		case 340: {
			if(time >= m_time && m_state == 340) {
				this->stopRobotMove();
				this->prepareThrowing(time);

				m_state = 350;
			}
			break;
		}
		case 350: {
			if(time >= m_time1 && m_state == 350) this->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(time >= m_time4 && m_state == 350) this->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(time >= m_time1 && time >= m_time4 && m_state == 350) {
				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashBoxName1);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + time;

				m_state = 360;
			}
			break;
		}
		case 360: {
			if(time >= m_time && m_state == 360) {
				this->stopRobotMove();
				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashBoxName1);

				double l_moveTime = goToObj(l_tpos, 50.0);
				m_time = l_moveTime + time;

				m_state = 370;
			}
			break;
		}
		case 370: {
			if(time >= m_time && m_state == 370) {
				this->stopRobotMove();
				double l_tpos[3];
				this->recognizeObjectPosition(l_tpos, m_trashBoxName1);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + time;

				m_state = 380;
			}
			break;
		}
		case 380: {  // throw trash and get back a bit
			if(time >= m_time && m_state == 380) {
				this->stopRobotMove();
				this->throwTrash();

				sleep(1);

				this->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 50.0/m_movingSpeed + time;

				m_state = 390;
			}
			break;
		}
		case 390: {  // recover robot arms
			if(time >= m_time && m_state == 390) {
				this->stopRobotMove();

				m_state = 0;
			}
			break;
		}
	}
	
	return refreshRateOnAction;
}

void RobotCommand::getPartsPosition(double l_pos[], std::string partName) {
   srvGetPartsPosition.request.part = partName;
   srvGetPartsPosition.request.name = "";
	if (serviceGetPartsPosition.call(srvGetPartsPosition)) {
	   l_pos[0] = srvGetPartsPosition.response.posX;
	   l_pos[1] = srvGetPartsPosition.response.posY;
	   l_pos[2] = srvGetPartsPosition.response.posZ;
	}  else {
	    ROS_ERROR("Failed to call service robot_000_get_parts_position");
	}
}

double RobotCommand::rotateTowardObj(double pos[])
{
	// "pos" means target position
	// get own position
	double ownPosition[3];
	
	getPartsPosition(ownPosition, "RARM_LINK2");
	double l_pos [3] = {pos[0]-ownPosition[0], pos[1]-ownPosition[1], pos[2]-ownPosition[2]};

	// ignore variation on y-axis
	l_pos[1] = 0;

	// get own rotation matrix
   srvGetRotation.request.name = "";
   double qw = 0;
   double qy = 0;
   if (serviceGetRotation.call(srvGetRotation)) {
      qw = srvGetRotation.response.qW;
      qy = srvGetRotation.response.qY;
   } else {
      ROS_ERROR("Failed to call service robot_000_get_rotation");
   }
   
	double theta = 2*acos(fabs(qw));
   
	if(qw*qy < 0) theta = -1.0*theta;

	// rotation angle from z-axis to x-axis
	srvGetAngleRotation.request.axis = "z";
	srvGetAngleRotation.request.x = l_pos[0];
	srvGetAngleRotation.request.y = l_pos[1];
	srvGetAngleRotation.request.z = l_pos[2];
	double tmp = 0;
	if (serviceGetAngleRotation.call(srvGetAngleRotation)) {
	   tmp = srvGetAngleRotation.response.angle;
	} else {
	    ROS_ERROR("Failed to call service robot_000_get_angle_rotation");
	}
	double targetAngle = acos(tmp);

	// direction
	if(l_pos[0] > 0) targetAngle = -1.0 * targetAngle;
	targetAngle += theta;

	double angVelFac = 3.0;
	double l_angvel = m_angularVelocity/angVelFac;

	if(targetAngle == 0.0) {
		return 0.0;
	} else {
		// circumferential distance for the rotation
		double l_distance = m_distance*M_PI*fabs(targetAngle)/(2.0*M_PI);

		// Duration of rotation motion (micro second)
		double l_time = l_distance / (m_movingSpeed/angVelFac);

		// Start the rotation
		if(targetAngle > 0.0) {
		   this->setWheelVelocity(l_angvel, -l_angvel);
		} else{
		   this->setWheelVelocity(-l_angvel, l_angvel);
		}

		return l_time;
	}
	return 0.1;
}

void RobotCommand::recognizeObjectPosition(double l_tpos[], std::string obj) {
   srvGetObjPosition.request.name = obj;
   if (serviceGetObjPosition.call(srvGetObjPosition)) {
      l_tpos[0] = srvGetObjPosition.response.posX;
      l_tpos[1] = srvGetObjPosition.response.posY;
      l_tpos[2] = srvGetObjPosition.response.posZ; 
   } else {
      ROS_ERROR("Failed to call service robot_000_get_obj_position");
   }
}

double RobotCommand::getTime() {
   if(serviceGetTime.call(srvGetTime)) {
      return srvGetTime.response.time;
   } else {
      ROS_ERROR("Failed to call service robot_000_get_time");
      return -1;
   }
}

double RobotCommand::goToObj(double pos[], double range) 
{
	// get own position
	double robotCurrentPosition[3];
	this->getPartsPosition(robotCurrentPosition, "RARM_LINK2");

	// pointing vector for target
	double l_pos [] = {pos[0]-robotCurrentPosition[0], pos[1]-robotCurrentPosition[1], pos[2]-robotCurrentPosition[2]};

	// ignore y-direction
	l_pos[1] = 0;

	// measure actual distance
	double length = sqrt(l_pos[0]*l_pos[0] + l_pos[1]*l_pos[1] + l_pos[2]*l_pos[2]);
	double distance = length - range;

	// start moving
	this->setWheelVelocity(m_angularVelocity, m_angularVelocity);

	// time to be elapsed
	double l_time = distance / m_movingSpeed;
	return l_time; //1 : ok, 1.3 : ok, 6.3 : ko
}


void RobotCommand::neutralizeArms(double evt_time)
{
	double angleJoint1 = this->getJointAngle("RARM_JOINT1")*180.0/(M_PI);
	double angleJoint4 = this->getJointAngle("RARM_JOINT4")*180.0/(M_PI);
	double thetaJoint1 = - 20 - angleJoint1;
	double thetaJoint4 = - 180 - angleJoint4;
	if (thetaJoint4 < 0) {
	   this->setJointVelocity("RARM_JOINT4", -m_jointVelocity, 0.0);
	} else {
	   this->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);
	}

	if(thetaJoint1 < 0) this->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	else this->setJointVelocity("RARM_JOINT1", m_jointVelocity, 0.0);

	m_time1 = DEG2RAD(abs(thetaJoint1))/ m_jointVelocity + evt_time;
	m_time4 = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity + evt_time;
}

void RobotCommand::setJointVelocity(std::string jointName, double angularVelocity, double max) {
   msgSetJointVelocity.jointName = jointName;
   msgSetJointVelocity.angularVelocity = angularVelocity;
   msgSetJointVelocity.max = max;
   robot_000_setJointVelocity_pub.publish(msgSetJointVelocity);
}

double RobotCommand::getJointAngle(std::string nameJoint) {
   srvGetJointAngle.request.nameArm = nameJoint;
   if (serviceGetJointAngle.call(srvGetJointAngle)) {
      return srvGetJointAngle.response.angle;
   } else {
      ROS_ERROR("Failed to call service robot_000_get_joint_angle");
   }
}

double RobotCommand::goGraspingObject(double pos[])
{
	double l_time;
	double thetaJoint4 = 20.0;
	this->setJointVelocity("RARM_JOINT4", -m_jointVelocity, 0.0);

	l_time = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity;

	return l_time;
}

void RobotCommand::setWheelVelocity(double leftWheel, double rightWheel) {
   msgSetWheelVelocity.leftWheel = leftWheel;
   msgSetWheelVelocity.rightWheel = rightWheel;
   robot_000_setWheelVelocity_pub.publish(msgSetWheelVelocity);
}

void RobotCommand::prepareThrowing(double evt_time)
{
	double thetaJoint1 = 50.0;
	this->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	m_time1 = DEG2RAD(abs(thetaJoint1))/ m_jointVelocity + evt_time;

	double thetaJoint4 = 65.0;
	this->setJointVelocity("RARM_JOINT4", -m_jointVelocity, 0.0);
	m_time4 = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity + evt_time;

}

void RobotCommand::throwTrash()
{
	// get the part info. 
	msgReleaseObj.arm = "RARM_LINK7";
	robot_000_releaseObj_pub.publish(msgReleaseObj);
	
	// wait a bit
	sleep(1);

	// set the grasping flag to neutral
	m_grasp = false;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "robot_command");
	
	RobotCommand* robotCommand = new RobotCommand();
	double result = 0;
	
   while (ros::ok())//
   {
      result = robotCommand->loop();
      if (result == -1) {
         return 1;
      } else {
         ros::spinOnce();
         //loop_rate.sleep();
         ros::Duration(result).sleep();
      }
   }
   return 0;
}

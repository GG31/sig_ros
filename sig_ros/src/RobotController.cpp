#include "include/RobotController.hpp"

void RobotController::onInit(InitEvent &evt)
{
   SimObjController::init();
   myRobot = getRobotObj(myname());
   
   myRobot->getPosition(position);
   myRobot->getRotation(rotation);
   m_view = (ViewService*)connectToService("SIGViewer"); 
   
   ros::NodeHandle n;
   
   //Topics
   setWheel_sub = n.subscribe<sig_ros::SetWheel>(std::string(this->myname()) + "_setWheel", 1, &RobotController::setWheelCallback, this);
   setWheelVelocity_sub = n.subscribe<sig_ros::SetWheelVelocity>(std::string(this->myname()) + "_setWheelVelocity", 1, &RobotController::setWheelVelocityCallback, this);
   
   //Slam
   scan_pub = n.advertise<sensor_msgs::LaserScan>(std::string(this->myname()) + "_scan", 1000);
   //clock_pub = n.advertise<rosgraph_msgs::Clock>("clock", 50);
   
   serviceGetJointPosition = n.advertiseService(std::string(this->myname()) + "_get_joint_position", &RobotController::getJointPosition, this);
   serviceIK = n.advertiseService(std::string(this->myname()) + "_ik", &RobotController::ik, this);
   current_time = ros::Time::now();
   last_time = ros::Time::now();
      
   x = 0.0;
   y = 0.0;
   th = 0.0;

   vx = 0.1;
   vy = -0.1;
   vth = 0.1;
   
   switchVal = 0;
   /* Load the robot model */
   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
   /* Get a shared pointer to the model */
   kinematic_model = robot_model_loader.getModel();

   /* Get the configuration for the joints in the right arm of the PR2*/
   joint_model_group = kinematic_model->getJointModelGroup("left_arm"); 

    /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
   const Eigen::Affine3d end_effector_default_pose = kinematic_state->getGlobalLinkTransform("l_wrist_roll_link");



   std::map<std::string, double> list = myRobot->getAllJointAngles();
   Vector3d v;
   for (std::map<std::string, double>::iterator it=list.begin(); it!=list.end(); ++it) {
       myRobot->getJointPosition(v, it->first.c_str());
       std::cout << it->first << " " << it->second << " : " << v.x() << " " << v.y() << " " << v.z() << std::endl;
   }
   /*std::cout << "matrix" << std::endl;
   matrix* m = new matrix(4,4);
   std::cout << "rot" << std::endl;
   m->setRotation(PI/2);
   std::cout << m->display() << std::endl;
   matrix* m2 = new matrix(4,4);
   m2->setTranslation(3,2,4);
   std::cout << m2->display() << std::endl;
   matrix* m3 = m->mul(m2);

   std::cout << m3->display() << std::endl;
   Vector3d v1(2,1,1);
   Vector3d m1(m->mul(v1));
   
   std::cout << m1.x() << " " << m1.y() << " " << m1.z()  << std::endl;*/
   fillPositionArms("left"); 
   fillPositionArms("right"); 
//      Vector3d a(116.64, 93.87, 42.05);
//      positionArms.insert(std::pair<std::string, Vector3d>("left", a));
   //bag.open("/home/gg/catkin_ws/src/user/xml/2015-06-19-15-29-31.bag", rosbag::bagmode::Read);
   //std::cout << bag.getSize() << " " << bag.getFileName() << std::endl;
   
   //topics.push_back(std::string("/tf"));
   //topics.push_back(std::string("/scan"));
   //view.addQuery(bag, rosbag::TopicQuery(topics));
   /*rosbag::View view(bag, rosbag::TopicQuery(topics));
   std::cout << view.size() << std::endl;
   
   foreach(rosbag::MessageInstance const m, view)
    {
      std::cout << "foreach" << std::endl;
        tf::tfMessage::ConstPtr s = m.instantiate<tf::tfMessage>();
        if (s != NULL)
            std::cout << "tf" << std::endl;

        sensor_msgs::LaserScan::ConstPtr i = m.instantiate<sensor_msgs::LaserScan>();
        if (i != NULL)
         std::cout << "scan" << std::endl;
    }

    bag.close();*/
}

double RobotController::onAction(ActionEvent &evt) {
   //ros::Time time = ros::Time::now() + ros::Duration(0.01);
   /*rosgraph_msgs::Clock msg;
   msg.clock = time;
   clock_pub.publish(msg);
   std::cout << "time " << time << std::endl;
*/
   /*rosbag::View view(bag, rosbag::TopicQuery(topics));
   int i = 0;
   std::cout << view.size() << std::endl;
   foreach(rosbag::MessageInstance const m, view)
   {
      if (i == switchVal) {
         tf::tfMessage::ConstPtr s = m.instantiate<tf::tfMessage>();
         if (s != NULL) {
            tf::TransformBroadcaster br;
            std::cout << "send tf" << std::endl;
            br.sendTransform(s->transforms);
            switchVal++;
            break;
         }

         sensor_msgs::LaserScan::ConstPtr i = m.instantiate<sensor_msgs::LaserScan>();
         if (i != NULL) {
            std::cout << "send scan" << std::endl;
            scan_pub.publish(i);
            switchVal++;
            break;
         }
      }
      i++;
   }*/
   //calculateTransform(ros::Time(m_simulatorTime));
   //scan(ros::Time(m_simulatorTime));
   
   return SimObjController::onAction(evt);
}

void RobotController::calculateTransform(ros::Time time) {
   tf::TransformBroadcaster br;
   Vector3d positionNow;
   Rotation rotationNow;
   myRobot->getPosition(positionNow);
   myRobot->getRotation(rotationNow);
   const dReal* quat = rotation.q();
   const dReal* quatNow = rotationNow.q();
   //std::cout << "pos : " << position.x() << " " << position.y() << " " << position.z() << " : " << positionNow.x() << " " << positionNow.y() << " " << positionNow.z() << std::endl;
   //std::cout << "rot : " << quatNow[1] - quat[1] << " " << quatNow[2] - quat[2] << " " << quatNow[3] - quat[3] << " " << quatNow[0] - quat[0] << std::endl;
   
   geometry_msgs::TransformStamped transforms;
   
   transforms.header.stamp = ros::Time::now();//m_simulatorTime;
   transforms.header.frame_id = "odom";
   transforms.child_frame_id = "base_link";
   transforms.transform.translation.x = positionNow.x() - position.x();
   transforms.transform.translation.y = positionNow.y() - position.y();
   transforms.transform.translation.z = positionNow.z() - position.z();
   
   //const dReal* quat = rotation.q();
   //const dReal* quatNow = rotationNow.q();
   transforms.transform.rotation.x = quatNow[1] - quat[1];
   transforms.transform.rotation.y = quatNow[2] - quat[2];
   transforms.transform.rotation.z = quatNow[3] - quat[3];
   transforms.transform.rotation.w = quatNow[0] - quat[0];
   br.sendTransform(transforms);
   
   
   /*transforms.header.stamp = ros::Time::now();//m_simulatorTime;
   transforms.header.frame_id = "base_footprint";
   transforms.child_frame_id = "base_link";
   transforms.transform.translation.x = 0;
   transforms.transform.translation.y = 0;
   transforms.transform.translation.z = 1;
 
   transforms.transform.rotation.x = 0;
   transforms.transform.rotation.y = 0;
   transforms.transform.rotation.z = 0;
   transforms.transform.rotation.w = 0;
   
   
   
   br.sendTransform(transforms);//tf::StampedTransform(transform, ros::Time(m_simulatorTime), "odom", "base_link"));*/
   
   position = positionNow;
   rotation = rotationNow;
}

void RobotController::scan(ros::Time time) {
   // Calculate vertical angle view (radian) of a camera.  
   double fovy = myRobot->getCamFOV() * PI / 180.0;  
   // Get aspect ratio  
   double ar = myRobot->getCamAS(); 
   // Calculate horizontal angle view (degree) of a camera.  
   double fovx = 2 * atan(tan(fovy*0.5)*ar) * 180.0 / PI; 
   unsigned char distance = 255;    
   if(m_view != NULL) {    
      // Get distance vector along a horizontal plane  
      ViewImage *img = m_view->distanceSensor1D();    
      char *buf = img->getBuffer();    
      // Get length of the data array  
      int length = img->getBufferLength();    

      float angle_increment = fovx/320 * (PI/180);
      float range_min = 0;
      float range_max = 255;
      sensor_msgs::LaserScan scan;
      scan.header.stamp = ros::Time(1.0);//m_simulatorTime;
      scan.header.frame_id = "base_laser_link";//"base_laser_link";
      scan.scan_time = 0.0;
      scan.angle_min = -(fovx/2 * PI/180);
      scan.angle_max = fovx/2 * PI/180;
      scan.angle_increment = fovx/320 * (PI/180);
      scan.time_increment = 0.0; //(1 / laser_frequency) / (num_readings);
      scan.range_min = 0.0;
      scan.range_max = 255.0;
    
      scan.ranges.resize(length);
      scan.intensities.resize(length);
      for (int i = 0; i < length; i++) {
         unsigned char c = buf[length - 1 - i];
         double d = c;
         scan.ranges[i] = d;
         scan.intensities[i] = 1.0;
      }
      scan_pub.publish(scan);
      
      //LOG_MSG(("theta = %.1f, distance = %d",theta, distance));    
      delete img;    
   }      
}

void RobotController::fillPositionArms(std::string arm) {
   std::string letter = getLetter("left");
   Vector3d posShoulderInit;
   myRobot->getJointPosition(posShoulderInit, (letter + "ARM_JOINT1").c_str());
   Vector3d posWristInit;
   myRobot->getJointPosition(posWristInit, (letter + "ARM_JOINT5").c_str());
   Vector3d posElbowInit;
   myRobot->getJointPosition(posElbowInit, (letter + "ARM_JOINT4").c_str());
   float lArm1 = norm(posShoulderInit, posElbowInit);
   Vector3d posElbow(posShoulderInit.x(), posShoulderInit.y(), posShoulderInit.z() + lArm1);
   
   float lArm2 = norm(posWristInit, posElbowInit); 
   Vector3d posWrist1(posElbow.x(), posElbow.y(), posElbow.z() + lArm2);
   
   /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
   char a = std::tolower(letter.c_str()[0]);
   std::string b = "_wrist_roll_link";
   /* Find the default pose for the end effector */
   kinematic_state->setToDefaultValues();
   const Eigen::Affine3d end_effector_default_pose = kinematic_state->getGlobalLinkTransform(a + b);
   Eigen::Affine3d end_effector_pose = Eigen::Translation3d(0.0, 0.0, 0.0) * end_effector_default_pose;
   ROS_INFO_STREAM("End effector position:\n" << end_effector_pose.translation());

   // use IK to get joint angles satisfyuing the calculated position 
   bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
   if (found_ik)
   {
      moveit_msgs::DisplayRobotState msg;
      robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
      std::cout << "angles : " << msg.state.joint_state.position[16] << " " << msg.state.joint_state.position[18] << std::endl;
      Vector3d posWrist = pointAfterRotation(posWrist1, posElbow, msg.state.joint_state.position[18]);
      positionArms.insert(std::pair<std::string, Vector3d>(arm, posWrist));
      std::cout << "position originale " << posWrist.x()<< " " << posWrist.y() << " " << posWrist.z() << std::endl;
   } 
}

float RobotController::norm(Vector3d v1, Vector3d v2) {
   return sqrt(pow(v1.x()-v2.x(), 2) + pow(v1.y()-v2.y(),2) + pow(v1.z() - v2.z(),2));
}

Vector3d RobotController::pointAfterRotation(Vector3d point, Vector3d center, float angle) {
   matrix* mtn = new matrix();
   mtn->setTranslation(-center.x(), -center.y(), -center.z());
   matrix* rot = new matrix();
   rot->setRotation(angle);
   matrix* res = rot->mul(mtn);
   mtn->reverseTranslation();
   matrix* res1 = mtn->mul(res);
   return res1->mul(point);
}

bool RobotController::getJointPosition(sig_ros::getPartsPosition::Request &req, sig_ros::getPartsPosition::Response &res) {
   Vector3d vec;
   if (req.name == "") {
      myRobot->getJointPosition(vec, req.part.c_str());
   } else {
      RobotObj *ent = getRobotObj(req.name.c_str());
      if (ent == NULL) {
         return false;
      }
      ent->getJointPosition(vec, req.part.c_str());
   }
   res.posX = vec.x();
   res.posY = vec.y();
   res.posZ = vec.z();
   return true;

}
/*****************************Callback topic************************/

void RobotController::setWheelCallback(const sig_ros::SetWheel::ConstPtr& wheel)
{
   m_radius = wheel->wheelRadius;           // radius of the wheel
	m_distance = wheel->wheelDistance; 
   myRobot->setWheel(m_radius, m_distance);
}

void RobotController::setWheelVelocityCallback(const sig_ros::SetWheelVelocity::ConstPtr& wheel)
{
   myRobot->setWheelVelocity(wheel->leftWheel, wheel->rightWheel);
}

/*****************************End callback topic************************/

/*****************************Services************************/
bool RobotController::ik(sig_ros::ik::Request &req, sig_ros::ik::Response &res) {
   /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

   /* Get the configuration for the joints in the right arm of the PR2*/
   //joint_model_group = kinematic_model->getJointModelGroup("left_arm"); 
   /* Find the default pose for the end effector */
   kinematic_state->setToDefaultValues();

//   std::string letter = req.arm == "left" ? "L" : req.arm == "right" ? "R" : return false;
   std::string letter = getLetter(req.arm);
   if (letter == "") return false;

   char a = std::tolower(letter.c_str()[0]);
   std::string b = "_wrist_roll_link";
   const Eigen::Affine3d end_effector_default_pose = kinematic_state->getGlobalLinkTransform(a + b);
   Vector3d pos(req.x, req.y, req.z);
   // If the position given is SIGVerse's
   if (req.position == "relative") {
      Vector3d posArmNow;
      myRobot->getJointPosition(posArmNow, (letter + "ARM_JOINT5").c_str());
      pos += posArmNow;
   }
   if (req.position == "absolute" || req.position == "relative") {
      std::cout << "pos asked " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
      std::cout << "pos origin " << positionArms.find(req.arm)->second.x() << " " << positionArms.find(req.arm)->second.y() << " " << positionArms.find(req.arm)->second.z() << std::endl;
      pos -= positionArms.find(req.arm)->second;
      std::cout << "pos diff " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
      pos.z(0.1 * pos.z() / 1.77);//0.1 -> 1.77
      pos.y(0.01 * pos.y() / 1.1); //0.05 -> 5.4564
      pos.x(0);
      std::cout << "pos calculé " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
   }
   std::cout << req.position.c_str() << " " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
   // calculate a position for the end effector 
   Eigen::Affine3d end_effector_pose =

      Eigen::Translation3d(pos.z(), pos.x(), pos.y()) * end_effector_default_pose;
   ROS_INFO_STREAM("End effector position:\n" << end_effector_pose.translation());

   // use IK to get joint angles satisfyuing the calculated position 
   bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
   if (found_ik)
   {
      moveit_msgs::DisplayRobotState msg;
      robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
      for (int i=0; i<44; i++) {
         std::cout << msg.state.joint_state.position[i] << " ";
      }
      std::cout << std::endl;
      myRobot->setJointAngle((letter + "ARM_JOINT1").c_str(), msg.state.joint_state.position[16] - PI/2);
      myRobot->setJointAngle((letter + "ARM_JOINT4").c_str(), msg.state.joint_state.position[18]);
      res.done = true;
   } else {
      ROS_INFO_STREAM("Could not solve IK for pose\n" << end_effector_pose.translation());
      res.done = false;
   }

   return true;
}

std::string RobotController::getLetter(std::string arm) {
   if (arm == "left") {
      return "L";
   } else if (arm == "right") {
      return "R";
   } else {
      return "";
   }
}

/*****************************End methods************************/
extern "C"  RobotController * createController ()
{
   return new RobotController;
}

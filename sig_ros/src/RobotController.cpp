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
   
   current_time = ros::Time::now();
   last_time = ros::Time::now();
      
   x = 0.0;
   y = 0.0;
   th = 0.0;

   vx = 0.1;
   vy = -0.1;
   vth = 0.1;
   
   switchVal = 0;
   
   std::map<std::string, double> list = myRobot->getAllJointAngles();
   Vector3d v;
   for (std::map<std::string, double>::iterator it=list.begin(); it!=list.end(); ++it) {
       myRobot->getJointPosition(v, it->first.c_str());
       std::cout << it->first << " " << it->second << " : " << v.x()-100.16 << " " << v.y() << " " << v.z()-16 << std::endl;
   }
   fillPositionArms(); 
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

void RobotController::fillPositionArms() {
   Vector3d v;
   myRobot->getJointPosition(v, "LARM_JOINT5");
   positionArms.insert(std::pair<std::string, Vector3d>("left", v));
   myRobot->getJointPosition(v, "RARM_JOINT5");
   positionArms.insert(std::pair<std::string, Vector3d>("right", v));
   std::cout << "pos " << positionArms.find("left")->second.x() << " " << positionArms.find("left")->second.y() << " " << positionArms.find("left")->second.z() << std::endl;
   std::cout << "pos " << positionArms.find("right")->second.x() << " " << positionArms.find("right")->second.y() << " " << positionArms.find("right")->second.z() << std::endl;
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

/*****************************Methods************************/
bool RobotController::ik(sig_ros::ik::Request &req, sig_ros::ik::Response &res) {
   /* Load the robot model */
   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
   /* Get a shared pointer to the model */
   robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

   /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

   /* Get the configuration for the joints in the right arm of the PR2*/
   const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("left_arm"); 
   /* Find the default pose for the end effector */
   kinematic_state->setToDefaultValues();

   const Eigen::Affine3d end_effector_default_pose = kinematic_state->getGlobalLinkTransform("l_wrist_roll_link");
   std::string shoulder = "";
   std::string elbow = "";
   if (req.arm == "right") {
      shoulder = "LARM_JOINT1";
      elbow = "LARM_JOINT4";
   } else if (req.arm == "left") {
      shoulder = "RARM_JOINT1";
      elbow = "RARM_JOINT4";
   } else {
      return false;
   }
   Vector3d pos(req.x, req.y, req.z);
   // If the position given is SIGVerse's
   if (req.position == "absolute") {
      pos -= positionArms.find(req.arm)->second;
   } else if (req.position == "relative") {
      Vector3d v;
      if (req.arm == "left") {
         myRobot->getJointPosition(v, "LARM_JOINT5");
      } else {
         myRobot->getJointPosition(v, "RARM_JOINT5");
      }
      pos -= v;
   }
   std::cout << req.position.c_str() << " " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
   // calculate a position for the end effector 
   Eigen::Affine3d end_effector_pose =

      Eigen::Translation3d(req.x, req.y, req.z) * end_effector_default_pose;
   ROS_INFO_STREAM("End effector position:\n" << end_effector_pose.translation());

   // use IK to get joint angles satisfyuing the calculated position 
   bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_pose, 10, 0.1);
   if (found_ik)
   {
      moveit_msgs::DisplayRobotState msg;
      robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
   /*   for (int i=0; i<44; i++) {
         std::cout << msg.state.joint_state.position[i] << " ";
      }
      std::cout << std::endl;*/
      myRobot->setJointAngle(shoulder.c_str(), msg.state.joint_state.position[16] - PI/2);
      myRobot->setJointAngle(elbow.c_str(), msg.state.joint_state.position[18]);
      res.done = true;
   } else {
      ROS_INFO_STREAM("Could not solve IK for pose\n" << end_effector_pose.translation());
      res.done = false;
   }
   return true;
}
/*****************************End methods************************/
extern "C"  RobotController * createController ()
{
   return new RobotController;
}

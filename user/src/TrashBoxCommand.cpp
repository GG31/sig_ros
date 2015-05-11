#include "TrashBoxCommand.hpp"

TrashBoxCommand::TrashBoxCommand() {
   init();
} 
  
void TrashBoxCommand::init() {

   
  /*m_my = getObj(myname());*/
   getAllEntities(m_entities);
  
   //Topics
	setJointVelocity_pub = n.advertise<sig_ros::SetJointVelocity>("trashbox_0_setJointVelocity", 1000);
	releaseObj_pub = n.advertise<sig_ros::ReleaseObj>("trashbox_0_releaseObj", 1000);
	setAxisAndAngle_pub = n.advertise<sig_ros::SetAxisAndAngle>("trashbox_0_setAxisAndAngle", 1000);
	setPosition_pub = n.advertise<sig_ros::SetPosition>("trashbox_0_setPosition", 1000);
	onRecvMsg_sub = n.subscribe<sig_ros::MsgRecv>("trashbox_0_onRecvMsg", 1, &TrashBoxCommand::onMsgRecvCallback, this);
	onCollision_sub = n.subscribe<sig_ros::OnCollision>("trashbox_0_onCollisionMsg", 1, &TrashBoxCommand::onCollisionCallback, this);
	//Srv
	serviceGetTime = n.serviceClient<sig_ros::getTime>("trashbox_0_get_time");
	serviceGetObjPosition = n.serviceClient<sig_ros::getObjPosition>("trashbox_0_get_obj_position");
	serviceGetPartsPosition = n.serviceClient<sig_ros::getPartsPosition>("trashbox_0_get_parts_position");
	serviceGetRotation = n.serviceClient<sig_ros::getRotation>("trashbox_0_get_rotation");
	serviceGetAngleRotation = n.serviceClient<sig_ros::getAngleRotation>("trashbox_0_get_angle_rotation");
	serviceGetJointAngle = n.serviceClient<sig_ros::getJointAngle>("trashbox_0_get_joint_angle");
   serviceGraspObj = n.serviceClient<sig_ros::graspObj>("trashbox_0_grasp_obj");
   serviceCheckService = n.serviceClient<sig_ros::checkService>("trashbox_0_check_service");
   serviceGetCollisionStateOfMainPart = n.serviceClient<sig_ros::getCollisionStateOfMainPart>("trashbox_0_get_collision_state_of_main_part");
   serviceGetEntities = n.serviceClient<sig_ros::getEntities>("trashbox_0_get_entities");
   serviceIsGrasped = n.serviceClient<sig_ros::isGrasped>("trashbox_0_is_grasped");
  
  
  
  m_ref = false;
  retValue = 0.5;
  roboName = "robot_000";

  colState = false;

  // ゴミ箱
  tboxSize_x  = 20.0;
  tboxSize_z  = 40.5; 
  tboxMin_y    = 40.0;
  tboxMax_y    = 1000.0;
}

double TrashBoxCommand::loop() {
// サービスが使用可能か定期的にチェックする  
   bool available = checkService("RobocupReferee");  

   if(!available && m_ref) m_ref = false;

   // 使用可能  
   else if(available && !m_ref){  
   // サービスに接続  
      m_ref = connectToService("RobocupReferee");  
   }  

   double myPos[3];
   getObjPosition(myPos, "");
  
 /*
  // 自分の位置取得
  Vector3d myPos;
  m_my->getPosition(myPos);*/

  // If you are in a collision , check whether the collision is continuing
   if (colState) {
      bool state = getCollisionStateOfMainPart();

      // To return to the state it does not have collision
      if (!state) colState = false;
   }
  
   int entSize = m_entities.size();
   for (int i = 0; i < entSize; i++) {

       // ロボットまたはゴミ箱の場合は除く
      if (m_entities[i] == "robot_000"  ||
         m_entities[i] == "bluetrashbox" ||
         m_entities[i] == "greentrashbox" ||
         m_entities[i] == "redtrashbox") {
         continue;
      }
    // エンティティ取得
//   SimObj *ent = getObj(m_entities[i].c_str());

      double tpos[3];
      getObjPosition(tpos, m_entities[i].c_str());

       // Vector connecting the garbage from the trash
      double vec [] = {tpos[0] - myPos[0], tpos[1] - myPos[1], tpos[2] - myPos[2]};
      //Vector3d vec(tpos.x()-myPos.x(), tpos.y()-myPos.y(), tpos.z()-myPos.z());
       
       // ゴミがゴミ箱の中に入ったかどうか判定
      if (abs(vec[0]) < tboxSize_x/2.0 &&
         abs(vec[2]) < tboxSize_z/2.0 &&
         tpos[1] < tboxMax_y     &&
         tpos[1] > tboxMin_y     ) {

         // ゴミがリリースされているか確認
         std::string name = m_entities[i].c_str();
         if (isGrasped(name)) {/*!ent->getIsGrasped()*///) {

            // ゴミを捨てる
            tpos[1] = tpos[1] / 2;
            tpos[0] = myPos[0];
            tpos[2] = myPos[2];
            setAxisAndAngle(name, 1.0, 0.0, 0.0, 0.0);
            setPosition(name, tpos[0], tpos[1], tpos[2]);
            usleep(500000);
            tpos[1] = 0.0;
            setPosition(name, tpos[0], tpos[1], tpos[2]);
            /*ent->setAxisAndAngle(1.0, 0.0, 0.0, 0.0);
            ent->setAxisAndAngle(1.0, 0.0, 0.0, 0.0);
            ent->setPosition(tpos);
            ent->setPosition(tpos);
            ent->setPosition(tpos);
            usleep(500000);
            tpos.y(0.0);
            ent->setPosition(tpos);
            ent->setPosition(tpos);
            ent->setPosition(tpos);*/

            std::string msg;
         // ゴミが所定のゴミ箱に捨てられているかチェック
         // リサイクル
            //if(strcmp(myname(), "trashbox_0") == 0){
            // 空のペットボトルのみ点が入る
               if (name == "petbottle_1" ||
                  name == "petbottle_2" ||
                  name == "petbottle" ||
                  name == "mayonaise_1") {
                  msg = "RobocupReferee/Clean up succeeded" "/1000";
               } else{
                  msg = "RobocupReferee/Clean up failed" "/-600";
               }
            /*}*/ /*else if(strcmp(myname(), "trashbox_1") == 0) { //Burning garbage
               // 燃えるゴミに入れるべきものは無い
               msg = "RobocupReferee/Clean up failed" "/-600";
            } else if(strcmp(myname(), "trashbox_2") == 0) {// 缶瓶
               if(strcmp(ent->name(), "can_0") == 0 ||
                  strcmp(ent->name(), "can_1") == 0 ||
                  strcmp(ent->name(), "can") == 0 ||
                  strcmp(ent->name(), "can_3") == 0) {
                  msg = "RobocupReferee/Clean up succeeded" "/1000";
               }
               else {
                  msg = "RobocupReferee/Clean up succeeded" "/-600";
               }
            }*/

            if (!m_ref) {
               sendMsgToSrv(msg.c_str());
            }
            ROS_INFO("%s", msg.c_str());
         }
      }
   }
   return retValue;
} 

/*void TrashBoxCommand::onCollision(CollisionEvent &evt) { 
  // 衝突していない状態のときのみ衝突をチェック
  if(!colState) {
    const std::vector<std::string> & wname= evt.getWith();
    int csize = wname.size();
    for(int i = 0; i < csize; i++){
      // robotと衝突
      if(wname[i] == roboName){
	colState = true;
	std::string msg = "RobocupReferee/Collision with [" + std::string(myname()) + "]" "/-100";
	if(m_ref != NULL){
	  m_ref->sendMsgToSrv(msg.c_str());
	}
	else{
	  LOG_MSG((msg.c_str()));
	}
      }
    }
  }
} */ 

void TrashBoxCommand::onMsgRecvCallback(const sig_ros::MsgRecv::ConstPtr& msg)
{
   std::cout << msg->sender.c_str() << std::endl;
   std::cout << msg->content.c_str() << std::endl;
}

void TrashBoxCommand::onCollisionCallback(const sig_ros::OnCollision::ConstPtr& msg)
{
   /*if (!m_grasp && (msg->name == m_trashName1 || msg->name == m_trashName2) && msg->part == "RARM_LINK7") {
      srvGraspObj.request.obj = msg->name;
      srvGraspObj.request.part = msg->part;
      if (serviceGraspObj.call(srvGraspObj)) {
         if (srvGraspObj.response.ok) {
            m_grasp = true; 
         }
      } else {
         ROS_ERROR("Failed to call service robot_000_grasp_obj");
      }
   }*/
}

void TrashBoxCommand::getObjPosition(double l_tpos[], std::string obj) {
   srvGetObjPosition.request.name = obj;
   if (serviceGetObjPosition.call(srvGetObjPosition)) {
      l_tpos[0] = srvGetObjPosition.response.posX;
      l_tpos[1] = srvGetObjPosition.response.posY;
      l_tpos[2] = srvGetObjPosition.response.posZ; 
   } else {
      ROS_ERROR("Failed to call service trashbox_0_get_obj_position");
   }
}

bool TrashBoxCommand::getCollisionStateOfMainPart() {
   if (serviceGetCollisionStateOfMainPart.call(srvGetCollisionStateOfMainPart)) {
      return srvGetCollisionStateOfMainPart.response.collisionState;
   } else {
      ROS_ERROR("Failed to call service trashbox_0_get_collision_state_of_main_part");
   }
}

double TrashBoxCommand::checkService(std::string nameJoint) {
   srvCheckService.request.serviceName = nameJoint;
   if (serviceCheckService.call(srvCheckService)) {
      return srvCheckService.response.connected;
   } else {
      ROS_ERROR("Failed to call service trashbox_0_check_service");
   }
}

double TrashBoxCommand::connectToService(std::string nameJoint) {
   srvConnectToService.request.serviceName = nameJoint;
   if (serviceConnectToService.call(srvConnectToService)) {
      return srvConnectToService.response.connected;
   } else {
      ROS_ERROR("Failed to call service trashbox_0_connect_to_service");
   }
}

void TrashBoxCommand::getAllEntities(std::vector<std::string> entities) {
   if (serviceGetEntities.call(srvGetEntities)) {
      entities = srvGetEntities.response.entitiesNames;
   } else {
      ROS_ERROR("Failed to call service trashbox_0_get_entities");
   }
}

bool TrashBoxCommand::isGrasped(std::string entityName) {
   srvIsGrasped.request.name = entityName;
   if (serviceIsGrasped.call(srvIsGrasped)) {
      return srvIsGrasped.response.answer;
   } else {
      ROS_ERROR("Failed to call service trashbox_0_is_grasped");
   }
}

void TrashBoxCommand::setAxisAndAngle(std::string name, double ax, double ay, double az, double angle) {
   msgSetAxisAndAngle.name = name;
   msgSetAxisAndAngle.axisX = ax;
   msgSetAxisAndAngle.axisY = ay;
   msgSetAxisAndAngle.axisZ = az;
   msgSetAxisAndAngle.angle = angle;
   setAxisAndAngle_pub.publish(msgSetAxisAndAngle);
}

void TrashBoxCommand::setPosition(std::string name, double ax, double ay, double az) {
   msgSetPosition.name = name;
   msgSetPosition.posX = ax;
   msgSetPosition.posY = ay;
   msgSetPosition.posZ = az;
   setPosition_pub.publish(msgSetPosition);
}

bool TrashBoxCommand::sendMsgToSrv(std::string msg) {
   srvSendMsgToSrv.request.msg = msg;
   if (serviceSendMsgToSrv.call(srvSendMsgToSrv)) {
      return srvSendMsgToSrv.response.ok;
   } else {
      ROS_ERROR("Failed to call service trashbox_0_send_msg_to_srv");
   }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "trashBox_command");
	
	TrashBoxCommand* trashBox = new TrashBoxCommand();
	double result = 0;
   while (ros::ok())//
   {
      result = trashBox->loop();
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


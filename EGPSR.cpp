#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <algorithm>
#include <unistd.h>
#include <math.h>
#include <string>
//convert angle unit from degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

//ControllerのサブクラスMoveControllerの宣言します
class RobotController : public Controller {
public:

  void onInit(InitEvent &evt);
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt);
  void onCollision(CollisionEvent &evt);
  bool recognizeTrash(Vector3d &pos, std::string &name);
  std::string getPointedTrashName(std::string entName);

  // エージェントが指差している方向にあるオブジェクトの名前を取得します
  std::string getPointedObjectName(std::string entName);
  // 位置を指定しその方向に回転を開始し、回転終了時間を返します
  double rotateTowardObj(Vector3d pos, double vel, double now);
  // 位置を指定しその方向に進みます
  double goToObj(Vector3d pos, double vel, double range, double now);

private:
  // Takeitの対称から除外するオブジェクトの名前
  std::vector<std::string> exobj;
  std::vector<std::string> extrash;
  RobotObj *m_my;
  Vector3d m_tpos;

  int m_robotState;
  int m_state;

  Vector3d m_defaultNormalVector;	// エージェントの正面の方向ベクトル通常はz軸の正の方向と考える

  std::string m_pointedObject;  // 取りにいくオブジェクト名

	// pointed trash
  std::string m_pointedtrash;

  // onActionの戻り値
  double m_onActionReturn;

  // 1stepあたりの移動距離
  double m_speedDelta;

  // ゴミの名前
  std::string m_tname;
  // ゴミ候補オブジェクト
  std::vector<std::string> m_trashes;
  // ゴミ箱オブジェクト
  std::vector<std::string> m_trashboxs;

  // 車輪の角速度
  double m_vel;

  // 関節の回転速度
  double m_jvel;

  // 車輪半径
  double m_radius;

  // 車輪間距離
  double m_distance;

  // 移動終了時間
  double m_time;
  double m_time_LA1;
  double m_time_LA4;
  double m_time_RA1;
  double m_time_RA4;

  // 初期位置
  Vector3d m_inipos;

	Vector3d m_relayPoint0;
	Vector3d m_relayPoint1;
	Vector3d m_relayPoint2;
	Vector3d m_relayPoint3;
	Vector3d m_relayPoint4;
	Vector3d m_relayPoint5;
	Vector3d m_relayPoint6;

	Vector3d m_kitchenPoint;
	Vector3d m_livingroomPoint;
	Vector3d m_bedroomPoint;
	Vector3d m_lobbyPoint;

  // grasp中かどうか
  bool m_grasp;

  // ゴミ認識サービス
  BaseService *m_recogSrv;

  // サービスへのリクエスト複数回送信を防ぐ
  bool m_sended;

  std::string msg_ob;
  std::string msg_trash;


bool is_to_startpoint;
int next_state;

};

void RobotController::onInit(InitEvent &evt)
{
  m_robotState = 0;
  //エージェントの正面の定義はz軸の正の向きを向いていると仮定する
  m_defaultNormalVector = Vector3d(0,0,1);
  m_onActionReturn = 0.01;
  m_speedDelta = 2.0;

  m_my = getRobotObj(myname());

  // 初期位置取得
  m_my->getPartsPosition(m_inipos,"RARM_LINK2");

	/*m_relayPoint0 = Vector3d(0, 30, 0);
	m_relayPoint1 = Vector3d(0, 30, 350);
	m_relayPoint2 = Vector3d(-400, 30, 0);
	
	m_kitchenPoint = Vector3d(200, 30, 350);;
	m_bedroomPoint = Vector3d(-400, 30, 200);;
	m_lobbyPoint = Vector3d(-400, 30, -200);;*/
	m_relayPoint1 = Vector3d(0, 30,0);
	m_relayPoint2= Vector3d(0, 30, 320);
	m_relayPoint3 = Vector3d(200,30,350);
	m_relayPoint4 = Vector3d(-400,30, 0);
	m_relayPoint5 = Vector3d(-400,30,-130);
	m_relayPoint6 = Vector3d(400,30,0);

	m_kitchenPoint = Vector3d(380, 30, 320);;
	m_bedroomPoint = Vector3d(-400,30,200);;
	m_lobbyPoint = Vector3d(-300, 30, -180);;
	m_livingroomPoint =Vector3d(300,30,-130);;
  // 車輪間距離
  m_distance = 10.0;

  // 車輪半径
  m_radius  = 10.0;

  // 移動終了時間初期化
  m_time = 0.0;
  m_time_LA1 = 0.0;
  m_time_LA4 = 0.0;
  m_time_RA1 = 0.0;
  m_time_RA4 = 0.0;

  // 車輪の半径と車輪間距離設定
  m_my->setWheel(m_radius, m_distance);
  m_state = 0;

  srand((unsigned)time( NULL ));

  // 車輪の回転速度
  m_vel = 1.0;

  // 関節の回転速度
  m_jvel = 1.0;

  // grasp初期化
  m_grasp = false;
  m_recogSrv = NULL;
  m_sended = false;

  // ここではゴミの名前が分かっているとします
  m_trashes.push_back("petbottle_1");
  m_trashes.push_back("can_0");
  m_trashes.push_back("apple");

  // ゴミ箱登録
  m_trashboxs.push_back("trashbox_0");
  m_trashboxs.push_back("trashbox_1");
  m_trashboxs.push_back("trashbox_2");

	broadcastMsgToSrv("Waiting 'start' message\n");


is_to_startpoint=false;

}

double RobotController::onAction(ActionEvent &evt)
{
	switch(m_state){
		//initialization
		case 0: { // wait next message
if(is_to_startpoint)
{
is_to_startpoint=false;
m_state=next_state;
}
			break;


		}
		case 1:	{ // set pose for initialization
			broadcastMsgToSrv("Set initial pose\n");
			double angL1 =m_my->getJointAngle("LARM_JOINT1")*180.0/(M_PI);
			double angL4 =m_my->getJointAngle("LARM_JOINT4")*180.0/(M_PI);
			double angR1 =m_my->getJointAngle("RARM_JOINT1")*180.0/(M_PI);
			double angR4 =m_my->getJointAngle("RARM_JOINT4")*180.0/(M_PI);
			double thetaL1 = -0-angL1;
			double thetaL4 = -0-angL4;
			double thetaR1 = -0-angR1;
			double thetaR4 = -0-angR4;
			if(thetaL1<0) m_my->setJointVelocity("LARM_JOINT1", -m_jvel, 0.0);
			else m_my->setJointVelocity("LARM_JOINT1", m_jvel, 0.0);
			if(thetaL4<0) m_my->setJointVelocity("LARM_JOINT4", -m_jvel, 0.0);
			else m_my->setJointVelocity("LARM_JOINT4", m_jvel, 0.0);
			if(thetaR1<0) m_my->setJointVelocity("RARM_JOINT1", -m_jvel, 0.0);
			else m_my->setJointVelocity("RARM_JOINT1", m_jvel, 0.0);
			if(thetaR4<0) m_my->setJointVelocity("RARM_JOINT4", -m_jvel, 0.0);
			else m_my->setJointVelocity("RARM_JOINT4", m_jvel, 0.0);
			m_time_LA1 = DEG2RAD(abs(thetaL1))/ m_jvel + evt.time();
			m_time_LA4 = DEG2RAD(abs(thetaL4))/ m_jvel + evt.time();
			m_time_RA1 = DEG2RAD(abs(thetaR1))/ m_jvel + evt.time();
			m_time_RA4 = DEG2RAD(abs(thetaR4))/ m_jvel + evt.time();
			m_state++;
			break;
		}
		case 2: { // change pose for initialization
			if(evt.time() >= m_time_LA1) m_my->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time_LA4) m_my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time_RA1) m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time_RA4) m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time_LA1 && evt.time() >= m_time_LA4
			&& evt.time() >= m_time_RA1 && evt.time() >= m_time_RA4){

				m_state = 0;
			}




			break;
		}
		
		// move toward the kitchen
		case 10: { // set rotation for relay point
			broadcastMsgToSrv("Move toward the kitchen");
			m_time = rotateTowardObj(m_relayPoint1, m_vel, evt.time());
			m_state++;
			break;
		}
		case 11: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 12: { // set rotation for relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
			m_time = rotateTowardObj(m_relayPoint2, m_vel, evt.time());
			m_state++;
			}
			break;
		}
		case 13: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint2, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 14: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_kitchenPoint, m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 15: { // rotate toward kitchen point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_kitchenPoint, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 16: { // move toward kitchen point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				broadcastMsgToSrv("Room_reached");
				//broadcastMsgToSrv("Waiting a next message\n");
				m_state = 0;
			}
			break;
		}
		
		// move toward the lobby
		case 20: { // set rotation for relay point
			broadcastMsgToSrv("Move toward the lobby");
			m_time = rotateTowardObj(m_relayPoint1, m_vel, evt.time());
			m_state++;
			break;
		}
		case 21: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 22: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_relayPoint4, m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 23: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint4, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 24: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_relayPoint5, m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 25: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint5, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 26: { // move toward lobby point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_lobbyPoint, m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 27: { // rotate toward lobby point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_lobbyPoint, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 28: { // move toward kitchen point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				broadcastMsgToSrv("Room_reached");
				//broadcastMsgToSrv("Waiting a next message\n");
				m_state = 0;
			}
			break;
		}

		// move toward the bed room
		case 30: { // set rotation for relay point
			broadcastMsgToSrv("Move toward the bed room");
			m_time = rotateTowardObj(m_relayPoint1, m_vel, evt.time());
			m_state++;
			break;
		}
		case 31: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 32: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_relayPoint4, m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 33: { // rotate toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint4, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 34: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_bedroomPoint,  m_vel, evt.time());
				m_state++;
			}
			break;
		}
		case 35: { // rotate toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_bedroomPoint, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 36: { // move toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				broadcastMsgToSrv("Room_reached");
				//broadcastMsgToSrv("Task finished\n");
				m_state = 0;
			}
			break;
		}

		case 40: { // set rotation for relay point
			broadcastMsgToSrv("Move toward the living room");
			m_time = rotateTowardObj(m_relayPoint1, m_vel,  evt.time());
			m_state++;

			break;
		}
		case 41: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint1, m_vel*4, 0.0, evt.time());
				m_state++;

			}
			break;
		}
		case 42: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_relayPoint2,  m_vel, evt.time());
				m_state++;

			}
			break;
		}
		case 43: { // rotate toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint2, m_vel*4, 0.0, evt.time());
				m_state++;

			}
			break;
		}
		case 44: { // move toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_kitchenPoint,  m_vel, evt.time());
				m_state++;

			}
			break;
		}
		case 45: { // rotate toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_kitchenPoint, m_vel*4, 0.0, evt.time());
				m_state++;

			}
			break;
		}
		case 46: { // set rotation for relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
			m_time = rotateTowardObj(m_relayPoint6, m_vel,  evt.time());
			m_state++;
			}
			break;
		}
		case 47: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint6, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 48: { // set rotation for relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
			m_time = rotateTowardObj(m_livingroomPoint, m_vel,  evt.time());
			m_state++;
			}
			break;
		}
		case 49: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_livingroomPoint, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 50: { // move toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				broadcastMsgToSrv("Room_reached");
				//broadcastMsgToSrv("Task finished\n");
				m_state = 0;
			}
			break;
		}


		case 100:{
			if(m_inipos.x()<-200&&m_inipos.z()<-130){
				m_state=110;
				break;
			}
			else if(m_inipos.x()<-300&&m_inipos.z()>0){
				m_state=120;
				break;
			}
			else if(m_inipos.x()>0&&m_inipos.z()>270){
				m_state=130;
				break;
			}
			else if(m_inipos.x()>270&&m_inipos.z()<0){
				m_state=140;
				break;
			}
			m_state=0;
			break;			 
		}
		case 110: { // x z <0
			m_time = rotateTowardObj(m_relayPoint5, m_vel, evt.time());
			m_state++;
			break;
		}
		case 111: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint5, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 112: { // set rotation for relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
			m_time = rotateTowardObj(m_relayPoint4, m_vel, evt.time());
			m_state++;
			}
			break;
		}
		case 113: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint4, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 114: { // move toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				//broadcastMsgToSrv("Task finished\n");
				m_state = 0;
			}
			break;
		}

		case 120: { // x<0 z>0
			m_time = rotateTowardObj(m_relayPoint4, m_vel, evt.time());
			m_state++;
			break;
		}
		case 121: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint4, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 122: { // set rotation for relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				//broadcastMsgToSrv("Task finished\n");
				m_state = 0;
			}
			break;
		}
		
		case 130: { // x z>0
			m_time = rotateTowardObj(m_relayPoint2, m_vel, evt.time());
			m_state++;
			break;
		}
		case 131: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint2, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 132: { // set rotation for relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				//broadcastMsgToSrv("Task finished\n");
				m_state = 0;
			}
			break;
		}


		case 140: { // x>0 z <0
			m_time = rotateTowardObj(m_relayPoint6, m_vel, evt.time());
			m_state++;
			break;
		}
		case 141: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint6, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 142: { // set rotation for relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
			m_time = rotateTowardObj(m_kitchenPoint, m_vel, evt.time());
			m_state++;
			}
			break;
		}
		case 143: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_kitchenPoint, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 144: { // set rotation for relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
			m_time = rotateTowardObj(m_relayPoint2, m_vel, evt.time());
			m_state++;
			}
			break;
		}
		case 145: { // rotate toward relay point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = goToObj(m_relayPoint2, m_vel*4, 0.0, evt.time());
				m_state++;
			}
			break;
		}
		case 146: { // move toward bed room point
			if(evt.time() >= m_time){
				m_my->setWheelVelocity(0.0, 0.0);
				//broadcastMsgToSrv("Task finished\n");
				m_state = 0;
			}
			break;
		}




  }
  return m_onActionReturn;
}

void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();


	std::string msg=evt.getMsg();
	LOG_MSG(("message : %s", msg.c_str()));
	std::string message;
	message=msg.c_str();
	int numlocbegin=0,numlocend=0,numitbegin=0,numitend=0;
	while(message.find(' ',0)<100)//delete ' '
	{
		message=message.erase(message.find(' ',0),1);
	}

	while(message.find("the",0)<100)//delete'the'
	{
		message=message.erase(message.find("the",0),3);
	}
	for(int i=0;i<message.length();i++)
	{
		if(message.substr(i,7)=="bedroom")
		{
			msg="bedroom";
		}
		else if(message.substr(i,7)=="kitchen")
		{
			msg="kitchen";
		}
		else if(message.substr(i,5)=="lobby")
		{
			msg="lobby";
		}
		else if(message.substr(i,10)=="livingroom")
		{
			msg="livingroom";
		}
		else if(message.substr(i,5)=="start")
		{
			msg="start";
		}
	}

	if(msg == "start" && m_state ==0)
	{
		m_state = 1; // initial settings
	}
	else if(msg == "kitchen" && m_state ==0)
	{
//		
	m_state =100;
	is_to_startpoint=true;
	next_state=10;
		//m_state = 10; // move to the kitchen
	}
	else if(msg == "lobby" && m_state ==0)
	{
		//m_state =100;
	m_state =100;
	is_to_startpoint=true;
	next_state=20; // move to the lobby
	}
	else if(msg == "bedroom" && m_state ==0)
	{
		//m_state =100;
	m_state =100;
	is_to_startpoint=true;
	next_state=30; // move to the bed room
	}
	else if(msg == "livingroom" && m_state ==0)
	{
		//m_state =100;
	m_state =100;
	is_to_startpoint=true;
	next_state=40; // move to the living room
	}
	else
	{
		broadcastMsgToSrv("Task finished\n");
	}
}


std::string RobotController::getPointedObjectName(std::string entName)
{
  // 発話者の名前からSimObjを取得します
  SimObj *tobj = getObj(entName.c_str());

  // メッセージ送信者の左肘関節の位置を取得します
  Vector3d jpos;
  if(!tobj->getJointPosition(jpos, "RARM_JOINT4")) {
    LOG_ERR(("failed to get joint position"));
    return "";
  }
  // メッセージ送信者の左肘から左手首をつなぐベクトルを取得します
  Vector3d jvec;
  if(!tobj->getPointingVector(jvec, "RARM_JOINT4", "RARM_JOINT7")) {
    LOG_ERR(("failed to get pointing vector"));
    return "";
  }

  double distance = 0.0;
  std::string objName = "";

  // 全ゴミオブジェクトでループします
  int trashSize = m_trashes.size();
  for(int i = 0; i < trashSize; i++) {

  // エンティティの位置を取得します
  SimObj *obj = getObj(m_trashes[i].c_str());
  Vector3d objVec;
  obj->getPosition(objVec);

  // エンティティと左肘関節を結ぶベクトルを作成します
  objVec -= jpos;

  // cos角度が不の場合（指差した方向と反対側にある場合)は対象から外します
  double cos = jvec.angle(objVec);
  if(cos < 0)
    continue;

  // 指差した方向ベクトルまでの最短距離の計算
  double theta = acos(cos);
  double tmp_distance = sin(theta) * objVec.length();

  // 最小距離の場合は名前、距離を保存しておく
  if(tmp_distance < distance || distance == 0.0){
    distance = tmp_distance;
    objName = obj->name();
    }
  }
  // エンティティでループして最も近いオブジェクトの名前を取得する
  return objName;
}

std::string RobotController::getPointedTrashName(std::string entName)
{
  // 発話者の名前からSimObjを取得します
  SimObj *tobj = getObj(entName.c_str());

  // メッセージ送信者の左肘関節の位置を取得します
  Vector3d jpos;
  if(!tobj->getJointPosition(jpos, "RARM_JOINT4")) {
    LOG_ERR(("failed to get joint position"));
    return "";
  }
  // メッセージ送信者の左肘から左手首をつなぐベクトルを取得します
  Vector3d jvec;
  if(!tobj->getPointingVector(jvec, "RARM_JOINT4", "RARM_JOINT7")) {
    LOG_ERR(("failed to get pointing vector"));
    return "";
  }

  double distance = 0.0;
  std::string objName = "";

  // 全ゴミオブジェクトでループします
  int trashboxSize = m_trashboxs.size();
  for(int i = 0; i < trashboxSize; i++) {

  // エンティティの位置を取得します
  SimObj *obj = getObj(m_trashboxs[i].c_str());
  Vector3d objVec;
  obj->getPosition(objVec);

  // エンティティと左肘関節を結ぶベクトルを作成します
  objVec -= jpos;

  // cos角度が不の場合（指差した方向と反対側にある場合)は対象から外します
  double cos = jvec.angle(objVec);
  if(cos < 0)
    continue;

  // 指差した方向ベクトルまでの最短距離の計算
  double theta = acos(cos);
  double tmp_distance = sin(theta) * objVec.length();

  // 最小距離の場合は名前、距離を保存しておく
  if(tmp_distance < distance || distance == 0.0){
    distance = tmp_distance;
    objName = obj->name();
    }
  }
  // エンティティでループして最も近いオブジェクトの名前を取得する
  return objName;
}

bool RobotController::recognizeTrash(Vector3d &pos, std::string &name)
{
  // 候補のゴミが無い場合
  if(m_trashes.empty()){
    return false;
  }

  // ここでは乱数を使ってゴミを決定します
  int trashNum = rand() % m_trashes.size();

  // ゴミの名前と位置を取得します
  name = m_trashes[trashNum];
  SimObj *trash = getObj(name.c_str());

  // ゴミの位置取得
  trash->getPosition(pos);
  return true;
}

void RobotController::onCollision(CollisionEvent &evt)
{
  if (m_grasp == false){
    typedef CollisionEvent::WithC C;
    //触れたエンティティの名前を得ます
    const std::vector<std::string> & with = evt.getWith();
    // 衝突した自分のパーツを得ます
    const std::vector<std::string> & mparts = evt.getMyParts();
    //　衝突したエンティティでループします
    for(int i = 0; i < with.size(); i++){
      //右手に衝突した場合
      if(mparts[i] == "RARM_LINK7"){
        //自分を取得
        SimObj *my = getObj(myname());
        //自分の手のパーツを得ます
        CParts * parts = my->getParts("RARM_LINK7");
        if(parts->graspObj(with[i])){
          m_grasp = true;
        }
      }
    }
  }
}

double RobotController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
  // 自分の位置の取得
  Vector3d myPos;
  //m_my->getPosition(myPos);
  m_my->getPartsPosition(myPos,"RARM_LINK2");

  // 自分の位置からターゲットを結ぶベクトル
  Vector3d tmpp = pos;
  tmpp -= myPos;

  // y方向は考えない
  tmpp.y(0);

  // 自分の回転を得る
  Rotation myRot;
  m_my->getRotation(myRot);

  // エンティティの初期方向
  Vector3d iniVec(0.0, 0.0, 1.0);

  // y軸の回転角度を得る(x,z方向の回転は無いと仮定)
  double qw = myRot.qw();
  double qy = myRot.qy();

  double theta = 2*acos(fabs(qw));

  if(qw*qy < 0)
    theta = -1*theta;

  // z方向からの角度
  double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
  double targetAngle = acos(tmp);

  // 方向
  if(tmpp.x() > 0) targetAngle = -1*targetAngle;
  targetAngle += theta;

  if(targetAngle == 0.0){
    return 0.0;
  }
  else {
    // 回転すべき円周距離
    double distance = m_distance*M_PI*fabs(targetAngle)/(2*M_PI);

    // 車輪の半径から移動速度を得る
    double vel = m_radius*velocity;

    // 回転時間(u秒)
    double time = distance / vel;

    // 車輪回転開始
    if(targetAngle > 0.0){
      m_my->setWheelVelocity(velocity, -velocity);
    }
    else{
      m_my->setWheelVelocity(-velocity, velocity);
    }

    return now + time;
  }
}

// object まで移動
double RobotController::goToObj(Vector3d pos, double velocity, double range, double now)
{
  Vector3d myPos;
  //m_my->getPosition(myPos);
  m_my->getPartsPosition(myPos,"RARM_LINK2");

  // 自分の位置からターゲットを結ぶベクトル
  pos -= myPos;

  // y方向は考えない
  pos.y(0);

  // 距離計算
  double distance = pos.length() - range;

  // 車輪の半径から移動速度を得る
  double vel = m_radius*velocity;

  // 移動開始
  m_my->setWheelVelocity(velocity, velocity);

  // 到着時間取得
  double time = distance / vel;

  return now + time;
}

//自身のインスタンスをSIGVerseに返します
extern "C" Controller * createController() {
  return new RobotController;
}

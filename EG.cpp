#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <iostream>
#include <vector>
#include <ViewImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.1415926535797
#define TIMEOUT 0.5

#define __DEBUG

using namespace std;

enum location{Bedroom, Kitchen, Lobby, Livingroom};
Vector3d g_v3d_relayPoint[] = 
{
	Vector3d(0, 30, 0),
	Vector3d(0, 30, 320),
	Vector3d(200, 30, 350),
	Vector3d(-400, 30, 0),
	Vector3d(-400, 30, -130),
	Vector3d(400, 30, 0)
};
Vector3d g_v3d_goalPoint[] = 
{
	Vector3d(-400, 30, 200),
	Vector3d(380, 30, 320),
	Vector3d(-300, 30, -180),
	Vector3d(300, 30, -130)
};

double _dotProductXZ(Vector3d &v3d_v1, Vector3d &v3d_v2)
{
	return v3d_v1.x() * v3d_v2.x() + v3d_v1.z() * v3d_v2.z();
}

class MyController : public Controller 
{  
public:  
  	void onInit(InitEvent &evt);  
  	double onAction(ActionEvent&);  
  	void onRecvMsg(RecvMsgEvent &evt); 
  	void onCollision(CollisionEvent &evt); 

public:
	RobotObj *m_pRO_robot;
	
	double m_d_wheelRadius;				// 机器人轮子半径
	double m_d_wheelDistance;			// 机器人两轮间距离
	
	bool m_b_start;
	location m_l_location;
	
	Vector3d m_v3d_robotPos;
	Vector3d m_v3d_robotDir;
  	
public:
	void turnLeft();
	void turnRight();
	void turnBack();
	void stop();
	void moveDis(double d_dis);
	void moveTo(Vector3d v3d_goalPoint)
	{
		getRobotPos();
		getRobotDir();
		
		Vector3d v3d_diff;
		v3d_diff.x(v3d_goalPoint.x() - m_v3d_robotPos.x());
		v3d_diff.z(v3d_goalPoint.z() - m_v3d_robotPos.z());
		
		cout << "Diff = " << v3d_diff.x() << "\t" << v3d_diff.z() << endl;
		
		double d_angleRobot = atan2(m_v3d_robotDir.x(), m_v3d_robotDir.z());
		double d_angleGoal = atan2(v3d_diff.x(), v3d_diff.z());
		
		double d_angleDiff = d_angleGoal - d_angleRobot;
		
		cout << d_angleDiff << " # ";
		
		double d_diffDis;
		if(fabs(d_angleDiff - 0) < 0.2)
		{
			d_diffDis = _dotProductXZ(v3d_diff, m_v3d_robotDir);
			moveDis(d_diffDis);
		}
		else if(fabs(d_angleDiff - PI / 2) < 0.2)
		{
			turnLeft();
			moveDis(_dotProductXZ(v3d_diff, m_v3d_robotDir));
		}
		
		
		
	}
	void moveToBedroom()
	{
		moveTo(g_v3d_relayPoint[0]);
		moveTo(g_v3d_relayPoint[3]);
		moveTo(g_v3d_goalPoint[Bedroom]);
	}
	void moveToKitchen()
	{
		static int si_run = 0;
		if(si_run == 0)
		{
			moveTo(g_v3d_relayPoint[0]);
			si_run++;
		}
		if(si_run == 1)
		{
			moveTo(g_v3d_relayPoint[1]);
			si_run++;
		}
		if(si_run == 2)
		{
			moveTo(g_v3d_goalPoint[Kitchen]);
			si_run++;
		}
	}
	void moveToLobby()
	{
		moveTo(g_v3d_relayPoint[0]);
		moveTo(g_v3d_relayPoint[3]);
		moveTo(g_v3d_relayPoint[4]);
		moveTo(g_v3d_goalPoint[Lobby]);
	}
	void moveToLivingroom()
	{
		moveToKitchen();
		moveTo(g_v3d_relayPoint[5]);
		moveTo(g_v3d_goalPoint[Livingroom]);
	}
	void getRobotPos()
	{
		m_pRO_robot->getPosition(m_v3d_robotPos);
		cout << "Pos = " << m_v3d_robotPos.x() << "\t" << m_v3d_robotPos.z() << endl;
	}
	void getRobotDir()
	{
		m_pRO_robot->getCamDir(m_v3d_robotDir, 3);
		cout << "Dir = " << m_v3d_robotDir.x() << "\t" << m_v3d_robotDir.z() << endl;
	}
	
};  
  
void MyController::onInit(InitEvent &evt) 
{  
	m_pRO_robot = getRobotObj(myname());		// 得到机器人指针
	m_d_wheelRadius = 10.0;
	m_d_wheelDistance = 10.0;

	m_pRO_robot->setWheel(m_d_wheelRadius, m_d_wheelDistance);
	
	m_b_start = false;
	
	getRobotPos();
	getRobotDir();
	
}  
  
double MyController::onAction(ActionEvent &evt) 
{
	switch(m_l_location)
	{
	case Kitchen:
		moveToKitchen();
		broadcastMsgToSrv("Room_reached");
		broadcastMsgToSrv("Give_up");
		break;
	}  
 	return TIMEOUT;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) 
{
	string s_sender = evt.getSender();
	string s_msg = evt.getMsg();
	
	if(s_msg == "Task_start" && m_b_start == false)
	{
		m_b_start = true;
		cout << "--> start" << endl;
		return;
	}
	
	string s_message = s_msg.c_str();
	
	// delete ' '
	while(s_message.find(' ', 0) < s_message.length())
	{
		s_message = s_message.erase(s_message.find(' ', 0), 1);
	}
	cout << "\t" << s_message << endl;
	// delete'the'
	while(s_message.find("the", 0) < s_message.length())
	{
		s_message = s_message.erase(s_message.find("the", 0), 3);
	}
	cout << "\t\t" << s_message << endl;
	if(s_message.find("bedroom", 0) < s_message.length())
	{
		m_l_location = Bedroom;
		cout << "--> bedroom" << endl;
	}
	if(s_message.find("kitchen", 0) < s_message.length())
	{
		m_l_location = Kitchen;
		cout << "--> kitchen" << endl;
	}
	if(s_message.find("lobby", 0) < s_message.length())
	{
		m_l_location = Lobby;
		cout << "--> lobby" << endl;
	}
	if(s_message.find("livingroom", 0) < s_message.length())
	{
		m_l_location = Livingroom;
		cout << "--> livingroom" << endl;
	}
	/*
	
	else
	{
		broadcastMsgToSrv("Task_finished\n");
	}*/
}  

void MyController::onCollision(CollisionEvent &evt) 
{ 
}
void MyController::turnLeft()
{
	double SCALE = 1;
	m_pRO_robot->setWheelVelocity(-PI / 4 / SCALE, PI / 4 / SCALE);
	sleep(1.0 * SCALE);
	m_pRO_robot->setWheelVelocity(0, 0);
	Vector3d v3d_oldDir = m_v3d_robotDir;
	m_v3d_robotDir.x(v3d_oldDir.z());
	m_v3d_robotDir.z(-v3d_oldDir.x());
	cout << "turn left" << endl;
}
void MyController::turnRight()
{
	double SCALE = 1;
	m_pRO_robot->setWheelVelocity(PI / 4 / SCALE, -PI / 4 / SCALE);
	sleep(1.0 * SCALE);
	m_pRO_robot->setWheelVelocity(0, 0);
	Vector3d v3d_oldDir = m_v3d_robotDir;
	m_v3d_robotDir.x(-v3d_oldDir.z());
	m_v3d_robotDir.z(v3d_oldDir.x());
	cout << "turn right" << endl;
}
void MyController::turnBack()
{
	double SCALE = 2;
	m_pRO_robot->setWheelVelocity(-PI / 2 / SCALE, PI / 2 / SCALE);
	sleep(1.0 * SCALE);
	m_pRO_robot->setWheelVelocity(0, 0);
	Vector3d v3d_oldDir = m_v3d_robotDir;
	m_v3d_robotDir.x(-v3d_oldDir.x());
	m_v3d_robotDir.z(-v3d_oldDir.z());
	cout << "turn back" << endl;
}
void MyController::stop()
{
	m_pRO_robot->setWheelVelocity(0, 0);
}
void MyController::moveDis(double d_dis)
{
	m_pRO_robot->setWheelVelocity(d_dis / 10, d_dis / 10);
	sleep(1.0);
	stop();
}
extern "C" Controller * createController() 
{  
  return new MyController;  
}  


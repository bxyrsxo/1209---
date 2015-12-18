//#include "System.h"
#include <stdlib.h>
#include <conio.h>	//kbhit
#include <string.h>
#include <fstream>
#include <sstream>

#include "Tcp6D.h"
#include <vector>
#include "TimerHandler.h"

// Robotics Library
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/math/Cubic.h>



boost::shared_ptr< rl::kin::Kinematics > kin[2];
Tcp6D tcp[2];
Eigen::Vector3d teach[2][10]; //vkneading
Eigen::Vector3d push[2][10];  //pressing
Eigen::Vector3d teach2[2][10]; //hkneading
Eigen::Vector3d rubbing_center[2][10]; //rubbing
Eigen::Vector3d current_p[2];
Eigen::Vector3d target_p[2];
Eigen::Vector3d virtualpoint=Eigen::Vector3d(0.01,0.08,0);

// todo: replace by enum variable 
const int Cartesian_mode = 0;
const int Joint_mode = 1;

int ControlMode = Cartesian_mode;

bool pushswitch=0;
int teach_count=0;
int play_count=0;
int state=0;
double dst[2]={0};
int timeindex=0;
float time_set[2][10]={0};
bool teachflag=false, mode1=false, mode2=false, mode3=false;

const int ESC_KEY = 27;
char cKey;
char sequence[]="666785";
bool repeat=false;
int repeat_c=0;
int lock_elbow=0;
using namespace std;

void keyboardcontrol(char key);
void display();

void Teach_vKneading();   // vertical kneading
void Teach_Pressing();    // pressing
void Teach_hKneading();   // horizontal kenading
void Teach_Rubbing();     // rubbing

void Play_vKneading();
void Play_Pressing();
void Play_hKneading();
void Play_Rubbing();

void _cdecl wmain( int argc, wchar_t **argv, wchar_t **envp )
{
		//------------------------------------------------------------------------------------------------------------

	// for periodic timer code
    LARGE_INTEGER  liPeriod;   // timer period
    HANDLE         hTimer;     // timer handle

    //  RTX periodic timer code:
    //  TO DO: Set default timer period to your desired time.
    //         The period needs to be an even multiple of the HAL period found in the control panel.

    liPeriod.QuadPart = SERVOLOOP_TIME * 10000;  // 10000 = 1ms

	Initial_IMPCard( ARM_L );    //ARM_L==0
	Initial_IMPCard( ARM_R );  //ARM_R==1
		
	// Create a periodic timer
	if (! (hTimer = RtCreateTimer(
									NULL,            // security
									0,               // stack size - 0 uses default
									TimerHandler,    // timer handler
									NULL,            // NULL context (argument to handler)
									RT_PRIORITY_MAX, // priority
									CLOCK_FASTEST) 
		  )
	   )      // RTX HAL timer
	{
		// TO DO:  exception code here
		// RtWprintf(L"RtCreateTimer error = %d\n",GetLastError());
		ExitProcess(1);
	}

	//讀取手臂組態.xml檔
	boost::shared_ptr< rl::kin::Kinematics > kinematics_L(rl::kin::Kinematics::create("massage.xml"));
	boost::shared_ptr< rl::kin::Kinematics > kinematics_R(rl::kin::Kinematics::create("massage.xml"));
	kin[0] = kinematics_L;
	kin[1] = kinematics_R;

	// initial tool center point
	tcp[0].input( 0.3, 0.35, 0, -90, 0, 180);
	tcp[1].input( 0.3, 0.35, 0, -90, 0, 180);
	

	RtSetTimerRelative( hTimer, &liPeriod, &liPeriod);

	//state=0 cartesian space control  state=1 teach mode  state=2 cartesian space control&積分器開
	bool excute=true;
	while(excute)
	{   
		 //   //這段是在run的時候按space可以讓機器停下來，並切換到state=1
			if (_kbhit())   
			{
				cKey = _getch();
				if (cKey == ' ') 
					{
						system("PAUSE");

					}
			}

			//這段是在寫重複動作，如果 repeat_c == repeat == 0 表示沒有要重複，顯示選單
			if (repeat==true ) 
			{
				if (sequence[repeat_c]=='5') cKey='5';
				if (sequence[repeat_c]=='6') cKey='6';
				if (sequence[repeat_c]=='7') cKey='7';
				if (sequence[repeat_c]=='8') cKey='8';
				if (sequence[repeat_c]=='\0') 
				{
					cout<<"end"<<endl;
					repeat=false;
					repeat_c=0;
				}
					repeat_c++;
				
			}
			else // repeat_c == repeat 
			{
				//system("CLS");
				display();
				std::cin>>cKey;	
				
			}
			
			switch (cKey)
			{
				case '0': //僅留重力補償
					state=1;  //only gravity
					break;
				case 'q': //Quit
					excute=false;
					break;
				case '1':  //teach mode1
					Teach_vKneading();
					break;
				case '2': //teach mode2
					Teach_Pressing();
					break;		
				case '3':
					Teach_hKneading();
					break;
				case '4':
					Teach_Rubbing();
					break;
				case '5':
					Play_Rubbing();
					break;
				case '6':  //play mode1
					Play_vKneading();
					break;
				case '7':  //play mode2
					Play_Pressing();
					break;
				case '8':
					Play_hKneading();
					break;
				case '9': //重複
					repeat=true;
					break;
				case 'z': //回到initial position
					tcp[0].input( 0.3, 0.35, 0, 0, 0, 0);
					tcp[1].input( 0.3, 0.35, 0, 0, 0, 0);
					lock_elbow=3;
					state=0;
					break;
				case 'x':
					lock_elbow=0;
					break;
			}

	}//end while( true )
//----------------------------------------------------------------------------------------------
		
	Close_IMPCard( ARM_L );
	Close_IMPCard( ARM_R );

	RtDeleteTimer( hTimer );
	printf("Close System\n");

	ExitProcess(0);
}

void keyboardcontrol(char key)
{
		target_p[0]=current_p[0];
		
		switch (key)
		{
		case '8':
			{
				target_p[0](1)=target_p[0](1)+0.01;
				tcp[0].input( target_p[0](0), target_p[0](1), target_p[0](2), 0, 0, 0);
				state=2;
				break;
			}
		case '5':
			{
				target_p[0](1)=target_p[0](1)-0.01;
				tcp[0].input( target_p[0](0), target_p[0](1), target_p[0](2), 0, 0, 0);
				state=2;
				break;
			}
		case '4':
			{
				target_p[0](2)=target_p[0](2)-0.01;
				tcp[0].input( target_p[0](0), target_p[0](1), target_p[0](2), 0, 0, 0);
				state=2;
				break;
			}
		case '6':
			{
				target_p[0](2)=target_p[0](2)+0.01;
				tcp[0].input( target_p[0](0), target_p[0](1), target_p[0](2), 0, 0, 0);
				state=2;
				break;
			}
		case '1':
			{
				target_p[0](0)=target_p[0](0)+0.01;
				tcp[0].input( target_p[0](0), target_p[0](1), target_p[0](2), 0, 0, 0);
				state=2;
				break;
			}
		case '7':
			{
				target_p[0](0)=target_p[0](0)-0.01;
				tcp[0].input( target_p[0](0), target_p[0](1), target_p[0](2), 0, 0, 0);
				state=2;
				break;
			}
		default:
			{
				state=1;
				break;
			}

		}
}

void display()
{
	std::cout<<"[1] teach vertical kneading points"<<std::endl;
	std::cout<<"[2] teach pressing points"<<std::endl;
	std::cout<<"[3] teach horizontal kneading points"<<std::endl;
	std::cout<<"[4] teach rubbing points"<<std::endl;
	std::cout<<"[5] play vertical kneading"<<std::endl;
	std::cout<<"[6] play pressing"<<std::endl;
	std::cout<<"[7] play horizontal kneading"<<std::endl;
	std::cout<<"[8] play rubbing"<<std::endl;
	std::cout<<"[9] play"<<std::endl;
	std::cout<<"[z] initial point"<<std::endl;
	std::cout<<"[q] Quit"<<std::endl;
	std::cout<<">>"<<std::endl;
}

void Teach_vKneading()   // vertical kneading
{
	state=1;  //only gravity
	for (int i=0; i<=2; i++)// 0是第一點 1是via point 2是第二點 3是via point
	{
		std::cout<<"press enter to save" <<i<<"th point (total 3 point)"<<std::endl;
		while(1)
		{
			if (_kbhit())
			{ 
				cKey =_getch();
				keyboardcontrol(cKey);
				if (cKey == 13)
				{
					if (i!=2)
					{
						teach[0][i]=current_p[0]+virtualpoint;
						teach[1][i]=current_p[1]+virtualpoint;
					}
					else
					{
						teach[0][i]=current_p[0];
						teach[1][i]=current_p[1];
					}
					break;
				}
			}
		}
	}
	teach[0][3]=teach[0][0]+Eigen::Vector3d(0,0,-0.05);
	teach[1][3]=teach[1][0]+Eigen::Vector3d(0,0,+0.05);
	teach[0][4]=teach[0][1]+Eigen::Vector3d(0,0,-0.05);
	teach[1][4]=teach[1][1]+Eigen::Vector3d(0,0,+0.05);
	teach[0][5]=teach[0][2];
	teach[1][5]=teach[1][2];

	teachflag=true;

}

void Teach_Pressing()    // pressing
{
	state=1;  //only gravity
						
	for (int i=0; i<=6; i=i+2)// 0是第一點 1是via point 2是第二點 3是via point
	{
		std::cout<<"press enter to save" <<i<<"th point (total 4 point)"<<std::endl;
		while(1)
		{
			if (_kbhit())
			{ 
				cKey =_getch();
				keyboardcontrol(cKey);
				if (cKey == 13)
				{
					push[0][i]=current_p[0];
					push[1][i]=current_p[1];
					push[0][i+1]=current_p[0];
					push[1][i+1]=current_p[1];
					push[0][i+1](1)=push[0][i+1](1)-0.1;
					push[1][i+1](1)=push[1][i+1](1)-0.1;
					break;
				}
			}
		}
	}
}

void Teach_hKneading()   // horizontal kenading
{
	state=1;  //only gravity
						
	for (int i=0; i<3; i++)// 
	{
		std::cout<<"press enter to save" <<i<<"th point (total 3 point)"<<std::endl;
		while(1)
		{
			if (_kbhit())
			{ 
				cKey =_getch();
				keyboardcontrol(cKey);
				if (cKey == 13)
				{
					teach2[0][i*3]=current_p[0]+virtualpoint;
					teach2[1][i*3]=current_p[1]+virtualpoint;

					teach2[0][i*3+1]=current_p[0]+virtualpoint;
					teach2[1][i*3+1]=current_p[1]+virtualpoint;
					teach2[0][i*3+1](2)=teach2[0][i*3+1](2)-0.1;
					teach2[1][i*3+1](2)=teach2[1][i*3+1](2)+0.1; //正負號不確定

					teach2[0][i*3+2]=current_p[0];
					teach2[1][i*3+2]=current_p[1];
					teach2[0][i*3+2](1)=teach2[0][i*3+2](1)-0.1;
					teach2[1][i*3+2](1)=teach2[1][i*3+2](1)-0.1; 

					break;
				}
			}
		}
	}

}

void Teach_Rubbing()
{
	state=1;  //only gravity
						
	for (int i=0; i<3; i++)// 
	{
		std::cout<<"press enter to save" <<i<<"th point (total 3 point)"<<std::endl;
		while(1)
		{
			if (_kbhit())
			{ 
				cKey =_getch();
				keyboardcontrol(cKey);
				if (cKey == 13)
				{
					rubbing_center[0][i*2]=current_p[0]+virtualpoint;
					rubbing_center[1][i*2]=current_p[1]+virtualpoint;

					rubbing_center[0][i*2+1]=current_p[0];
					rubbing_center[1][i*2+1]=current_p[1];
					rubbing_center[0][i*2+1](1)=rubbing_center[0][i*2+1](1)-0.08;
					rubbing_center[1][i*2+1](1)=rubbing_center[1][i*2+1](1)-0.08; //正負號不確定

					break;
				}
			}
		}
	}
	
	

}


void Play_vKneading()
{
	if (teachflag==false) 
		std::cout<<"no point"<<std::endl;
	else
	{
		//先計算時間
		int k=100*40; //gain值。讓距離乘上一個gain值當作時間限制，距離的單位應該是mm 這個參數要調調看
		time_set[0][0]=k*sqrt(pow(teach[0][0](0)-current_p[0](0),2)+pow(teach[0][0](1)-current_p[0](1),2)+pow(teach[0][0](2)-current_p[0](2),2));
		time_set[1][0]=k*sqrt(pow(teach[1][0](0)-current_p[1](0),2)+pow(teach[1][0](1)-current_p[1](1),2)+pow(teach[1][0](2)-current_p[1](2),2));
								
		for (int i=1;i<=5; i++)
		{
			time_set[0][i]=k*sqrt(pow(teach[0][i](0)-teach[0][i-1](0),2)+pow(teach[0][i](1)-teach[0][i-1](1),2)+pow(teach[0][i](2)-teach[0][i-1](2),2));
			time_set[1][i]=k*sqrt(pow(teach[1][i](0)-teach[0][i-1](1),2)+pow(teach[1][i](1)-teach[1][i-1](1),2)+pow(teach[1][i](2)-teach[1][i-1](2),2));

		}
								
		state = 0;
		timeindex=0;
		play_count=0;
	}
	
	while(1)
	{
		if (play_count<=5)
		{
			lock_elbow=1;
			tcp[1].input( teach[1][play_count](0) , teach[1][play_count](1), teach[1][play_count](2) , 0, 0, 0);
			tcp[0].input( teach[0][play_count](0) , teach[0][play_count](1), teach[0][play_count](2) , 0, 0, 0);

			if (timeindex>time_set[0][play_count])	//目前沒有分左右手時間，讓左右手一致
			{
				play_count++;			
				timeindex=0;
			}
		}
		else  //全部play完...
		{
			lock_elbow=0;
			play_count=0;
			state=1;
			break;
		}
	}
}

void Play_Pressing()
{
	//先計算時間
	int k=100*60; //gain值。讓距離乘上一個gain值當作時間限制，距離的單位應該是mm 這個參數要調調看
	time_set[0][0]=k*sqrt(pow(push[0][0](0)-current_p[0](0),2)+pow(push[0][0](1)-current_p[0](1),2)+pow(push[0][0](2)-current_p[0](2),2));
	time_set[1][0]=k*sqrt(pow(push[1][0](0)-current_p[1](0),2)+pow(push[1][0](1)-current_p[1](1),2)+pow(push[1][0](2)-current_p[1](2),2));

	for (int i=1;i<=7; i++)
	{
		time_set[0][i]=k*sqrt(pow(push[0][i](0)-push[0][i-1](0),2)+pow(push[0][i](1)-push[0][i-1](1),2)+pow(push[0][i](2)-push[0][i-1](2),2));
		time_set[1][i]=k*sqrt(pow(push[1][i](0)-push[0][i-1](1),2)+pow(push[1][i](1)-push[1][i-1](1),2)+pow(push[1][i](2)-push[1][i-1](2),2));
	}

	state=0;
	timeindex=0;
	play_count=0;

	while(1)
	{
		//先移動到位置
		lock_elbow=1;
		if (play_count<=7)
		{
				tcp[1].input( push[1][play_count](0) , push[1][play_count](1), push[1][play_count](2) , 0, 0, 0);
				tcp[0].input( push[0][play_count](0) , push[0][play_count](1), push[0][play_count](2) , 0, 0, 0);
				if (timeindex>time_set[0][play_count])
				{
					timeindex=0;
					if (play_count%2==0)pushswitch=true;
					play_count++;
				}
		}
		else //全部結束
		{
			lock_elbow=0;
			play_count=0;
			state=1;
			break;
		}
		//執行一段時間的按摩
		while (pushswitch)
		{
			if (timeindex>3140) // 這個時間必須跟sine wave的頻率乘起來為pi/2的倍數
			{
				pushswitch=0;
				timeindex=0;
			}
		}
	}	
}

void Play_hKneading()
{
	//先計算時間
	int k=100*50; //gain值。讓距離乘上一個gain值當作時間限制，距離的單位應該是mm 這個參數要調調看
	time_set[0][0]=k*sqrt(pow(teach2[0][0](0)-current_p[0](0),2)+pow(teach2[0][0](1)-current_p[0](1),2)+pow(teach2[0][0](2)-current_p[0](2),2));
	time_set[1][0]=k*sqrt(pow(teach2[1][0](0)-current_p[1](0),2)+pow(teach2[1][0](1)-current_p[1](1),2)+pow(teach2[1][0](2)-current_p[1](2),2));
	for (int i=1;i<=8;i++)
	{
		time_set[0][i]=k*sqrt(pow(teach2[0][i](0)-teach2[0][i-1](0),2)+pow(teach2[0][i](1)-teach2[0][i-1](1),2)+pow(teach2[0][i](2)-teach2[0][i-1](2),2));
		time_set[1][i]=k*sqrt(pow(teach2[1][i](0)-teach2[0][i-1](1),2)+pow(teach2[1][i](1)-teach2[1][i-1](1),2)+pow(teach2[1][i](2)-teach2[1][i-1](2),2));
	}
	state=0;
	timeindex=0;
	play_count=0;
			
	while(1)
	{
		lock_elbow=2;
		if (play_count<=8)
		{
			tcp[1].input( teach2[1][play_count](0) , teach2[1][play_count](1), teach2[1][play_count](2) , 0, 0, 0);
			tcp[0].input( teach2[0][play_count](0) , teach2[0][play_count](1), teach2[0][play_count](2) , 0, 0, 0);
			if (timeindex>time_set[0][play_count])	//目前沒有分左右手時間，讓左右手一致
			{
				play_count++;			
				timeindex=0;
			}
		}
		else  //全部play完...
		{
			lock_elbow=0;
			play_count=0;
			state=1;
			break;
		}
	}

}

void Play_Rubbing()
{
	state=0;

	double r = 0.03;
	
	for (int k=0;k<3;k++)
	{

		tcp[0].input( rubbing_center[0][k*2](0) + r, rubbing_center[0][k*2](1), rubbing_center[0][k*2](2) , 0, 0, 0);
		tcp[1].input( rubbing_center[1][k*2](0) + r, rubbing_center[1][k*2](1), rubbing_center[1][k*2](2) , 0, 0, 0);
		
		Sleep(2000);
		lock_elbow=1;
		double x, y, z;

		for( int i = 0; i < 8; i++)
		{
			double th = 45.0*i;
			x = r*cos(th);
			z = r*sin(th);
		
			y = 0.3*(-x+0.03);  // maximum while x=-0.03, y = 0.3*0.06=0.18

			tcp[0].input( rubbing_center[0][k*2](0) + x, rubbing_center[0][k*2](1) + y , rubbing_center[0][k*2](2) + z, 0, 0, 0);
			tcp[1].input( rubbing_center[1][k*2](0) + x, rubbing_center[1][k*2](1) + y , rubbing_center[1][k*2](2) + z, 0, 0, 0);
			
			Sleep(400);
		}

		tcp[0].input( rubbing_center[0][k*2+1](0), rubbing_center[0][k*2+1](1), rubbing_center[0][k*2+1](2) , 0, 0, 0);
		tcp[1].input( rubbing_center[1][k*2+1](0), rubbing_center[1][k*2+1](1), rubbing_center[1][k*2+1](2) , 0, 0, 0);
		
		Sleep(1000);
		lock_elbow=0;
	}
}

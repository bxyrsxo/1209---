#include "TimerHandler.h"

extern boost::shared_ptr< rl::kin::Kinematics > kin[2];
extern Tcp6D tcp[2];
extern int state;
extern Eigen::Vector3d current_p[2];
extern int timeindex;  //用來記錄play經過的時間
extern Eigen::Vector3d normal;
extern bool pushswitch;
extern int lock_elbow;
long lEncoder[2][6] = {0};
float joint_pos[2][6] = {0.0};
float last_joint_pos[2][6] = {0.0};
// last position and angular position
Eigen::MatrixXd OTG_X_Last = Eigen::MatrixXd::Zero(2, 6);
// last velocity and angular velocity
Eigen::MatrixXd OTG_V_Last = Eigen::MatrixXd::Zero(2, 6);

float Gcomp[2][4] = {0.0};
float updateTorque[2][6] = {0.0};
float Integator[2][6]={0}; //積分項
float fixed_joint2[2]={0};
float fixed_joint3[2]={0};
float elbow_integator=0;
extern int ControlMode;

// RTX periodic timer handler function
void RTFCNDCL TimerHandler( PVOID context )
{

	// read the current state of the motor angle (encoder counter)
	Read_Encoder( joint_pos, lEncoder , ARM_L);   
	Read_Encoder( joint_pos, lEncoder , ARM_R);

	if( ControlMode == 0)  // Cartesian mode
	{

		static int initial_count = 1; //程式啟動的時候要給OTG_X_Last和OTG_V_Last初始值
		if( initial_count)
		{
			setEncoder ();
			for( int j = 0; j < 2; j++)
			{
				OTG_X_Last(j, 0) = 0.3;
				OTG_X_Last(j, 1) = 0.35;
				OTG_X_Last(j, 2) = 0;
				OTG_X_Last(j, 3) = -90*rl::math::DEG2RAD;
				OTG_X_Last(j, 4) = 0;
				OTG_X_Last(j, 5) = 180*rl::math::DEG2RAD;

				for( int i = 0; i < 6; i++)
					OTG_V_Last(j, i) = 0;
			}
			initial_count = 0;
		}
		static bool initial_2pos=1; //鎖住的時候用的
		static bool initial_3pos=1;
		
		// forward kinematics of current state
		// velocity and angular velocity
		Eigen::MatrixXd x(2, 6), v(2,6);
		// 0: current state,  1: on-line generated state,  2: target state
		Eigen::Vector3d t0[2], t1[2], t2[2];  //translation
		Eigen::Matrix3d r0[2], r1[2], r2[2];  //rotation
		for( int j = 0; j < 2 ; j++)
		{
			Eigen::Vector3d dx, dth;
			// get current transformation matrix
			rl::math::Vector q(kin[j]->getDof());
		
			for ( int i = 0; i < kin[j]->getDof(); i++)
				q(i) = joint_pos[j][i];

			kin[j]->setPosition(q);
			kin[j]->updateFrames();
			rl::math::Transform T0 = kin[j]->forwardPosition();  //4x4 transformation matrix
		
			r0[j] = T0.rotation();
			t0[j] = T0.translation();
		
			// calculate the target transformation matrix
			r2[j] = Eigen::AngleAxisd( tcp[j].a*rl::math::DEG2RAD, Eigen::Vector3d::UnitZ())  
				   *Eigen::AngleAxisd( tcp[j].b*rl::math::DEG2RAD, Eigen::Vector3d::UnitY())
				   *Eigen::AngleAxisd( tcp[j].c*rl::math::DEG2RAD, Eigen::Vector3d::UnitX());
			t2[j] = Eigen::Vector3d( tcp[j].x, tcp[j].y, tcp[j].z);

			// calculate the on-line generated transformation matrix
			t1[j] = Eigen::Vector3d( OTG_X_Last(j, 0), OTG_X_Last(j, 1), OTG_X_Last(j, 2));
			r1[j] = Eigen::AngleAxisd( OTG_X_Last(j, 3), Eigen::Vector3d::UnitZ())
				   *Eigen::AngleAxisd( OTG_X_Last(j, 4), Eigen::Vector3d::UnitY())
				   *Eigen::AngleAxisd( OTG_X_Last(j, 5), Eigen::Vector3d::UnitX());

			dx = t2[j] - t1[j];

			x(j, 0) = t2[j](0) - t1[j](0);
			x(j, 1) = t2[j](1) - t1[j](1);
			x(j, 2) = t2[j](2) - t1[j](2);
			x(j, 3) = tcp[j].a*rl::math::DEG2RAD - OTG_X_Last(j, 3);
			x(j, 4) = tcp[j].b*rl::math::DEG2RAD - OTG_X_Last(j, 4);
			x(j, 5) = tcp[j].c*rl::math::DEG2RAD - OTG_X_Last(j, 5);
		}
		v = x/0.001;

		//static int ccc = 1;

		//if( ccc++ % 100 == 0)
		//{
		//	std::cout<<x<<std::endl;
		//	std::cout<<std::endl;
		//}

		// maximum velocity and maximum angular velocity
		const double vmax[6] = {0.5, 0.5, 0.5, 1, 1, 1};   // m/s
		// maximum acceleration and maximum angular acceleration
		const double amax[6] = {1, 1, 1, 2, 2, 2};
	
		// cartesian force
		rl::math::Vector torque[2];			
		rl::math::Vector TCP_V[2];			// current cartesian velocity

		// OTG here, three dimensional- x,y,z
		for( int j = 0; j < 2; j++)
		{
			rl::math::Vector OTG_X(6);
			rl::math::Vector OTG_V(6);			

			kin[j]->updateJacobian();
			rl::math::Matrix J = kin[j]->getJacobian();

			// current cartesian velocity
			rl::math::Vector dq(6);
			for( int i = 0; i < 6; i++)
				dq(i) = (joint_pos[j][i] - last_joint_pos[j][i])/0.001;		
			TCP_V[j] = J*dq;

			for( int i = 0; i < 6; i++)
			{
				OTG_X(i) = x(j, i);
				OTG_V(i) = v(j, i);
				if( fabs(OTG_X(i)) < 0.000001)
					OTG_X(i) = 0;
				double vel_dec  = sqrt( 2*amax[i]*fabs(OTG_X(i)));	// deceleration part of the velocity profile

				if( OTG_V(i) >= 0)
				{
					if( OTG_V(i) > vmax[i])
						OTG_V(i) = vmax[i];
					if( OTG_V(i) > vel_dec)
						OTG_V(i) = vel_dec;
					if( OTG_V(i) > OTG_V_Last(j,i) + amax[i]*0.001)
						OTG_V(i) = OTG_V_Last(j,i) + amax[i]*0.001;
				}
				else
				{
					if( OTG_V(i) < -vmax[i])
						OTG_V(i) = -vmax[i];
					if( OTG_V(i) < -vel_dec)
						OTG_V(i) = -vel_dec;
					if( OTG_V(i) < OTG_V_Last(j,i) - amax[i]*0.001)
						OTG_V(i) = OTG_V_Last(j,i) - amax[i]*0.001;
				}

				// update
				OTG_X(i) = OTG_X_Last(j,i) + 0.5*(OTG_V(i)+OTG_V_Last(j,i))*0.001;
				// record the last cartesian position and velocity
				OTG_X_Last(j,i) = OTG_X(i);
				OTG_V_Last(j,i) = OTG_V(i);
			}
		
			// update on-line generated state
			Eigen::Vector3d dx, dth;
			Eigen::Vector3d tcmd( OTG_X(0), OTG_X(1), OTG_X(2));
			// calcuate the target transformation matrix
			Eigen::Matrix3d rcmd;
			rcmd = Eigen::AngleAxisd( OTG_X(3), Eigen::Vector3d::UnitZ())
				   *Eigen::AngleAxisd( OTG_X(4), Eigen::Vector3d::UnitY())
				   *Eigen::AngleAxisd( OTG_X(5), Eigen::Vector3d::UnitX());

			Eigen::Matrix3d skew = rcmd*r0[j].transpose() - Eigen::Matrix3d::Identity();
			dth = Eigen::Vector3d( 0.5*(skew(2,1)-skew(1,2)), 0.5*(skew(0,2)-skew(2,0)), 0.5*(skew(1,0)-skew(0,1)));
			dx = tcmd - t0[j];

			rl::math::Vector delta_x(6);
			for( int i = 0; i < 3; i++)
			{
				delta_x(i) = dx(i);
				delta_x(i+3) = 0;  //dth沒有放進去，所以本來6*6的J矩陣只有用到左上的4*3
			}

			rl::math::Vector delta_v(6);
			for( int i = 0; i < 6; i++)
				delta_v(i) = OTG_V(i) - TCP_V[j](i);

			rl::math::Vector k(6), kx(6) ;
			const float d = 0.0;
	
			kx(0) = 200; //x
			kx(1) = 200; //y
			kx(2) = 600; //z
			kx(3) = 100; 
			kx(4) = 100;
			kx(5) = 100;
		
			float I_gain=0; 

			if (state==2) I_gain=1; //開積分器

			for( int i = 0; i < 6; i++)
			{
				Integator[j][i] +=delta_x(i);
				delta_x(i) = kx(i)*delta_x(i)+I_gain*Integator[j][i];
			}

			if (pushswitch == 1)
				{
				
					float massage_sin=abs(25*sin(double(0.006*timeindex)));   //80  0.002
					delta_x(0)=delta_x(0)+massage_sin*0.15;
					delta_x(1)=delta_x(1)+massage_sin*0.85;
					//delta_x(2)=delta_x(2)+puchswitch_sin*normal(2);
					//delta_x(1)=delta_x(1)+puchswitch_sin;
				}

			torque[j] = J.transpose()*( delta_x);

		
			if (lock_elbow==1)  //另一隻手的還沒寫
			{
				if (initial_2pos==1)
				{
					fixed_joint2[ARM_L]=joint_pos[ARM_L][1];
					fixed_joint2[ARM_R]=joint_pos[ARM_R][1];
					initial_2pos=0;
				}
				torque[j](1)=torque[j](1)+30*(fixed_joint2[j]-joint_pos[j][1]);  //越大鎖越緊

			}
			if (lock_elbow==2)
			{

				if (initial_3pos==1)
				{
					fixed_joint3[ARM_L]=joint_pos[ARM_L][2];
					fixed_joint3[ARM_R]=joint_pos[ARM_R][2];
					initial_3pos=0;
				}
				torque[j](2)=torque[j](2)+30*(fixed_joint3[j]-joint_pos[j][2]);  //越大鎖越緊
			}
			if (lock_elbow==3)
			{
				elbow_integator=elbow_integator+0.01;
				if (joint_pos[j][1] < 0) torque[j](1)=torque[j](1)+elbow_integator-abs(joint_pos[j][1]-last_joint_pos[j][1]) ;
				else if (joint_pos[j][1] >0 ) torque[j](1)=torque[j](1)-elbow_integator+abs(joint_pos[j][1]-last_joint_pos[j][1]);
				
				
			}

		if (j==ARM_L) //左手
		{
			k(0) = 2;
			k(1) = 1;
			k(2) = 1;   
			k(3) = 2;
			k(4) = 0;
			k(5) = 0;

		}
		else      //右手
		{
			k(0) = 3;
			k(1) = 1;
			k(2) = 2;   
			k(3) = 3;
			k(4) = 0;
			k(5) = 0;
		}
			for( int i = 0; i < 6; i++)
				torque[j][i] *= k(i);

		
		}

		// Gravity_Compensation
		Gravity_Compensator( ARM_L, Gcomp[0] );
		Gravity_Compensator( ARM_R, Gcomp[1] );

		//---------------------------------------------------------------------------------------------------------------------------------------------------
		for(int i=0; i<4; i++)
		{
			updateTorque[0][i] = Gcomp[0][i];
			updateTorque[1][i] = Gcomp[1][i];
		}
		if (state==0 || state==2) //cartesian space control
		{
			timeindex++;
			for( int j = 0; j < 2; j++)
				for( int i = 0; i < 6; i++)
					updateTorque[j][i] += torque[j](i);
		}
		if (state == 1) //在teach模式之下，必須手動更新OTG內的值 
		{
			initial_2pos=1;
			initial_3pos=1;
			for( int j = 0; j < 2; j++)
			{
				OTG_X_Last(j, 0) = t0[j](0);
				OTG_X_Last(j, 1) = t0[j](1);
				OTG_X_Last(j, 2) = t0[j](2);
				OTG_X_Last(j, 3) = 0;
				OTG_X_Last(j, 4) = 0;
				OTG_X_Last(j, 5) = 0;

				for( int i = 0; i < 6; i++)
					OTG_V_Last(j, i) = 0;
			}
		}
	
		current_p[0]=t0[0];  //更新current position
		current_p[1]=t0[1];  //更新current position

	}
	else if ( ControlMode == 1)   // joint mode
	{






	}

	// output command
	Output_Voltage( updateTorque, ARM_L );
	Output_Voltage( updateTorque, ARM_R );

	// record the last position data of joint
	for( int i = 0; i < 6; i++)
	{
		last_joint_pos[ARM_L][i] = joint_pos[ARM_L][i];		
		last_joint_pos[ARM_R][i] = joint_pos[ARM_R][i];
	}
	
} //end of void RTFCNDCL TimerHandler( PVOID context )


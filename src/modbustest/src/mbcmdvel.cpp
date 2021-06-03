#include <ros/ros.h>
#include <std_msgs/String.h>
#include <modbustest/readinputregisters.h>
#include <modbustest/WriteMultipleRegisters.h>
#include <sstream>
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <string>
#include <iostream>


//  RABBIT Dimensions
#define RABBIT_DIAMETER		0.100
#define RABBIT_AXLE_LENGTH		0.270
#define RABBIT_MAX_RPM			240

#define  RABBIT_MAX_LIN_VEL_MM_S	500
#define  RABBIT_MAX_ANG_VEL_RAD_S	20.94		//((2*pi)/60)*200 RPM = 20.94
//#define  RABBIT_MAX_RADIUS_MM		2000

//!  RABBIT max encoder counts
#define  RABBIT_TOTAL_ENCODER_COUNTS	1845.0
//!  RABBIT encoder pulses to meter constant
//Wheel Diameter 96mm
#define  RABBIT_PULSES_TO_M		0.00016755			//(2 * PI * Radius of wheel)/Encoder counts per revolution

#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif
#define index_count 6
#define index_start 0

int m1encoder_slaveid;
int m2encoder_slaveid;

int m1encoder_index;
int m2encoder_index;

unsigned long long int encoder1_counts = 0;
unsigned long long int encoder2_counts = 0;

ros::WallTime last_cmd_vel_received;
float left_speed_mm_s, right_speed_mm_s;
int ml_dir, mr_dir;

void modbusreadCallback(const modbustest::readinputregisters::ConstPtr& msg)
{
unsigned int slaveid = msg->slaveid;
unsigned int read_input_registers_count = msg->read_input_registers_count;
unsigned int read_input_registers_start_address = msg->read_input_registers_start_address;

unsigned int tmp1=0;
unsigned int tmp2=0;

	if(slaveid == m1encoder_slaveid)
	{
		encoder1_counts=msg->read_input_registers[m1encoder_index];
		//ROS_INFO("slave %i index:%i Data::%lli:%i:%i",slaveid, m1encoder_index, encoder1_counts,tmp1,tmp2);
	}
	if(slaveid == m2encoder_slaveid)
	{
		encoder2_counts=msg->read_input_registers[m2encoder_index];
		//ROS_INFO("slave %i index:%i Data::%lli:%i:%i",slaveid, m2encoder_index, encoder2_counts,tmp1,tmp2);
	}
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	//int left_speed_mm_s = (int)((cmd_vel->linear.x-RABBIT_AXLE_LENGTH*cmd_vel->angular.z/2)*1e3);	// Left wheel velocity in mm/s
	//int right_speed_mm_s = (int)((cmd_vel->linear.x+RABBIT_AXLE_LENGTH*cmd_vel->angular.z/2)*1e3);	// Right wheel velocity in mm/s

	left_speed_mm_s = (cmd_vel->linear.x-(RABBIT_AXLE_LENGTH*cmd_vel->angular.z)/2)*1e3; // Left wheel velocity in mm/s
	right_speed_mm_s = (cmd_vel->linear.x+(RABBIT_AXLE_LENGTH*cmd_vel->angular.z)/2)*1e3;// Right wheel velocity in mm/s

	//ROS_INFO("Cmd Vel: %f : %f",left_speed_mm_s, right_speed_mm_s);
	last_cmd_vel_received = ros::WallTime::now();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmdvel_node");
  ros::NodeHandle n;

  unsigned long long int left_encoder, right_encoder;
  unsigned long long int last_left_encoder, last_right_encoder;
  float dt;
  modbustest::WriteMultipleRegisters mb_motor_pub;

  float cmd_vel_time_out;
  bool cmd_vel_time_out_disable;
  float mlkp, mlkd, mlkw, mrkp, mrkd, mrkw, mrperr, mrderr, mrierr, mlperr, mlderr, mlierr;

  n.getParam("control/mlkp", mlkp);
  n.getParam("control/mlkd", mlkd);
  n.getParam("control/mlkw", mlkw);
  n.getParam("control/mrkp", mrkp);
  n.getParam("control/mrkd", mrkd);
  n.getParam("control/mrkw", mrkw);

  int motor_slaveid;
  n.getParam("mmotor/motor_slaveid", motor_slaveid);

  int motor_registers_count;
  n.getParam("mmotor/motor_registers_count", motor_registers_count);

  int motor_registers_start_address;
  n.getParam("mmotor/motor_registers_start_address", motor_registers_start_address);

  int motor_data[motor_registers_count];

  int ml_en_index, ml_dir_index, ml_pwm_index;
  n.param<int>("mmotor/ml_enindex", ml_en_index, 0);
  n.param<int>("mmotor/ml_dirindex", ml_dir_index, 1);
  n.param<int>("mmotor/ml_pwmindex", ml_pwm_index, 2);

  int mr_en_index, mr_dir_index, mr_pwm_index;
  n.param<int>("mmotor/mr_enindex", mr_en_index, 3);
  n.param<int>("mmotor/mr_dirindex", mr_dir_index, 4);
  n.param<int>("mmotor/mr_pwmindex", mr_pwm_index, 5);

  n.param<int>("mencoder/m1encoder_slaveid", m1encoder_slaveid, 51);
  n.param<int>("mencoder/m2encoder_slaveid", m2encoder_slaveid, 53);

  n.param<int>("mencoder/m1encoder_index", m1encoder_index, 7);
  n.param<int>("mencoder/m2encoder_index", m2encoder_index, 7);

  n.param<bool>("rabbit/cmd_vel_time_out_disable", cmd_vel_time_out_disable, false);
  n.param<float>("rabbit/cmd_vel_time_out", cmd_vel_time_out, 10);

  ros::Subscriber chatter_sub1 = n.subscribe("modbus_read_input_registers", 1,modbusreadCallback);
  ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmdVelReceived);
  ros::Publisher WriteMultipleRegisters_pub = n.advertise<modbustest::WriteMultipleRegisters>("modbus_writemultipleregisters", 1);

	ros::Rate r(25);

	ROS_INFO("Ready to send command velocity to rabbit base robot with timeout of %f and %s",cmd_vel_time_out, cmd_vel_time_out_disable ? "False":"True");



	float current_left_speed_mm_s, current_right_speed_mm_s;

	mlierr = 0;
	left_speed_mm_s = 0;
	right_speed_mm_s = 0;
	current_left_speed_mm_s = 0.0;
	current_right_speed_mm_s = 0.0;
	last_cmd_vel_received = ros::WallTime::now();

	ros::WallTime current_wall_time = last_cmd_vel_received;

	ros::Time current_time, last_ml_time, last_mr_time;
	current_time = ros::Time::now();
	last_ml_time = ros::Time::now();
	last_mr_time = ros::Time::now();

  	while(n.ok())
	{
		signed long int ldiff = 0;
		signed long int rdiff = 0;

		left_encoder= encoder1_counts;
		right_encoder= encoder2_counts;

		//Setting robot to come rest if cmd_vel fails to come within the specified timeout in secs
		if(cmd_vel_time_out_disable == false) {
		current_wall_time = ros::WallTime::now();
		//ROS_INFO("Command velocity timeout %f :: %f : %f",current_wall_time,last_cmd_vel_received, cmd_vel_time_out);
		if((current_wall_time - last_cmd_vel_received).toSec() > cmd_vel_time_out)
		{
		//rabbit_left->writeMotorSpeed(0);
		//rabbit_right->writeMotorSpeed(0);
		left_speed_mm_s = 0;
		right_speed_mm_s = 0;
		last_cmd_vel_received = current_wall_time;
		ROS_INFO("Command velocity timeout");
		}
		}

		if(left_encoder!= last_left_encoder)
		{
			if((last_left_encoder > left_encoder)&&(left_encoder>0))		ROS_INFO("ldiff ");

			if((last_left_encoder > left_encoder)&&(last_left_encoder > 32000 )&&(left_encoder < 1400))
			{
			ldiff = (abs(32768 - abs(last_left_encoder)) + abs(left_encoder));
			}
			else
			{
			ldiff = abs(abs(left_encoder) - abs(last_left_encoder));
			}
		last_left_encoder = left_encoder;
		}
		if(right_encoder!= last_right_encoder)
		{
		if((last_right_encoder > right_encoder)&&(right_encoder>0))		ROS_INFO("rdiff ");
			if((last_right_encoder > right_encoder)&&(last_right_encoder > 32000 )&&(right_encoder < 1400))
			{
			rdiff = (abs(32768 - abs(last_right_encoder)) + abs(right_encoder));
			}
			else
			{
			rdiff = abs(abs(right_encoder) - abs(last_right_encoder));
			}
		last_right_encoder = right_encoder;
		}

		//PID Control of Left motor
		if(abs(left_speed_mm_s) > 0)
		{
		current_time = ros::Time::now();
		dt = (current_time - last_ml_time).toSec();

		current_left_speed_mm_s  = (3.14*RABBIT_DIAMETER*(ldiff/RABBIT_TOTAL_ENCODER_COUNTS)*1000.0)/dt;

		unsigned int lpwm;

		mlperr = (left_speed_mm_s - current_left_speed_mm_s);
		mlderr = mlperr/dt;
		mlierr = 0;

		lpwm = mlkp * mlperr + mlkd * mlderr + mlkw * mlierr;

		lpwm = abs(lpwm);

		if(lpwm > 255) lpwm = 200;

		//Setting enable parameter for left motor
		motor_data[ml_en_index]  = 1;

		//Setting direction parameter for left motor
		if((left_speed_mm_s/abs(left_speed_mm_s)) < 0)	motor_data[ml_dir_index] = 0;
		else 						motor_data[ml_dir_index] = 1;

		//Setting pwm duty parameter for left motor
		motor_data[ml_pwm_index] = lpwm;
		last_ml_time = current_time;

		ROS_INFO("%f : %i ML:%li: %f : %f ME: %f : %f",dt, lpwm, ldiff, current_left_speed_mm_s, left_speed_mm_s, mlperr, mlderr);
		}
		else
		{
		//If speed value is zero then motor is disabled
		motor_data[ml_en_index]  = 0;
		motor_data[ml_dir_index] = 1;
		motor_data[ml_pwm_index] = 25;
		}

		//PID Control of Right motor
		if(abs(right_speed_mm_s) > 0)
		{
		current_time = ros::Time::now();
		dt = (current_time - last_mr_time).toSec();
		
		unsigned int rpwm;

		current_right_speed_mm_s = (3.14*RABBIT_DIAMETER*(rdiff/RABBIT_TOTAL_ENCODER_COUNTS)*1000.0)/dt;

		mrperr = (right_speed_mm_s - current_right_speed_mm_s);
		mrderr = mrperr/dt;
		mrierr = 0;

		rpwm = mrkp * mrperr + mrkd * mrderr + mrkw * mrierr;

		rpwm = abs(rpwm);

		if(rpwm > 255) rpwm = 200;

		//Setting enable parameter for right motor
		motor_data[mr_en_index]  = 1;

		//Setting direction parameter for right motor
		if((right_speed_mm_s/abs(right_speed_mm_s)) < 0) motor_data[mr_dir_index] = 0;
		else 						 motor_data[mr_dir_index] = 1;

		//Setting pwm duty parameter for right motor
		motor_data[mr_pwm_index] = rpwm;
		last_mr_time = current_time;

		ROS_INFO("%f : %i MR:%li : %f : %f ME: %f : %f",dt, rpwm, rdiff, current_right_speed_mm_s, right_speed_mm_s, mrperr, mrderr);
		}
		else
		{
		//If speed value is zero then motor is disabled
		motor_data[mr_en_index]  = 0;
		motor_data[mr_dir_index] = 1;
		motor_data[mr_pwm_index] = 25;
		}

		mb_motor_pub.write_multiple_registers.clear();

		mb_motor_pub.slaveid = motor_slaveid;
		mb_motor_pub.write_multiple_registers_count = motor_registers_count;
		mb_motor_pub.write_multiple_registers_start_address  = motor_registers_start_address;

		for(int i=index_start; i<index_count;i++)
		{
			long int data = motor_data[i];
			mb_motor_pub.write_multiple_registers.push_back(data);
			//ROS_INFO("(%i) Data: %ld", i, data);
		}

		//publish the robot velocity message		
		WriteMultipleRegisters_pub.publish(mb_motor_pub);
		ros::spinOnce();
		r.sleep();	
	}

  return 0;
}

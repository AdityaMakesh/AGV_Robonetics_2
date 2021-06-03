#include <ros/ros.h>
#include <modbustest/readinputregisters.h>
#include <modbustest/readholdingregisters.h>
#include <modbustest/WriteMultipleRegisters.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <modbustest/pose.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <sstream>
#include <string>


//  RABBIT Dimensions
#define RABBIT_DIAMETER		0.100
// Distance between wheel of robot
#define RABBIT_AXLE_LENGTH	0.280
#define RABBIT_MAX_RPM		255

#define  RABBIT_MAX_LIN_VEL_MM_S	500
#define  RABBIT_MAX_ANG_VEL_RAD_S	20.94		//((2*pi)/60)*200 RPM = 20.94
//#define  RABBIT_MAX_RADIUS_MM		2000

//!  RABBIT max encoder counts
#define  RABBIT_TOTAL_ENCODER_COUNTS	1845.0
//#define  RABBIT_1_ROT_ENCODER_COUNTS	1845
//!  RABBIT encoder pulses to meter constant  14760
//Wheel Diameter 100mm
// RABBIT_PULSES_TO_M = (2 * PI * 50)/1845
#define  RABBIT_PULSES_TO_M		0.000170			//(2 * PI * Radius of wheel)/Encoder counts per revolution

#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

int m1encoder_slaveid;
int m2encoder_slaveid;
int m3encoder_slaveid;
int m4encoder_slaveid;

int m1encoder_index;
int m2encoder_index;
int m3encoder_index;
int m4encoder_index;

int mvoltage_slaveid;
int mvoltage_index;
float avoltage;

unsigned long long int encoder1_counts = 0;
unsigned long long int encoder2_counts = 0;

int ml_slaveid, mr_slaveid;
int ml_index, mr_index;

ros::WallTime last_cmd_vel_received;
float left_speed_mm_s, right_speed_mm_s;
int ml_dir, mr_dir;

unsigned long long int left_encoder, right_encoder;
float odometry_x_, odometry_y_, odometry_yaw_;

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
	if(slaveid == mvoltage_slaveid)
	{
		avoltage = ((msg->read_input_registers[mvoltage_index]*5*3.33)/1024);
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

void modbusdataCallback(const modbustest::readholdingregisters::ConstPtr& msg)
{
unsigned int slaveid = msg->slaveid;
unsigned int read_input_registers_count = msg->read_holding_registers_count;
unsigned int read_input_registers_start_address = msg->read_holding_registers_start_address;
unsigned int dir = 0;

	if(slaveid == ml_slaveid)
	{
		dir = msg->read_holding_registers[ml_index];
		if(dir == 0)	ml_dir = -1;
		else		ml_dir =  1;
		//ROS_INFO("slave %i index:%i Data::%i:%i",slaveid, ml_index, ml_dir, dir);
	}
	if(slaveid == mr_slaveid)
	{
		dir = msg->read_holding_registers[mr_index];
		if(dir == 0)	mr_dir = -1;
		else		mr_dir =  1;
		//ROS_INFO("slave %i index:%i Data::%i:%i",slaveid, mr_index, mr_dir, dir);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rabbit_node");
	ros::NodeHandle n;

	std_msgs::Float32MultiArray mvel;
	float last_x, last_y, last_yaw;

	float vel_x, vel_y, vel_yaw;
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

	n.param<int>("mencoder/m1encoder_index", m1encoder_index, 8);
	n.param<int>("mencoder/m2encoder_index", m2encoder_index, 8);

	n.param<int>("mvoltage/slaveid", mvoltage_slaveid, 3);
	n.param<int>("mvoltage/index", mvoltage_index, 7);

	n.param<bool>("rabbit/cmd_vel_time_out_disable", cmd_vel_time_out_disable, false);
	n.param<float>("rabbit/cmd_vel_time_out", cmd_vel_time_out, 10);

	last_x =0;
	last_y = 0;
	last_yaw = 0;
	odometry_x_ = 0;
	odometry_y_ = 0;
	odometry_yaw_ = 0;
	last_left_encoder = 0;
	last_right_encoder = 0;

 	//Getting slave details, coil counts and starting address register details from ros parameters
	if((m1encoder_slaveid == 0) && (m2encoder_slaveid == 0))
	{
	ROS_ERROR("Error: Parameters for encoder - slave id  not loaded sucessfully!");
	return 0;
	}

	signed long int xldiff = 0;
	signed long int xrdiff = 0;

	float current_left_speed_mm_s, current_right_speed_mm_s;

	mlierr = 0;
	left_speed_mm_s = 0;
	right_speed_mm_s = 0;
	current_left_speed_mm_s = 0.0;
	current_right_speed_mm_s = 0.0;
	last_cmd_vel_received = ros::WallTime::now();

	ros::WallTime current_wall_time = last_cmd_vel_received;

	ros::Time current_time, last_time, last_ml_time, last_mr_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	last_ml_time = ros::Time::now();
	last_mr_time = ros::Time::now();


	modbustest::pose robot_pose;

	robot_pose.x = robot_pose.y = robot_pose.theta = 0;
	robot_pose.linear_velocity = robot_pose.angular_velocity = 0;

	std_msgs::Float32 mb_voltage_pub;

	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber chatter_sub1 = n.subscribe("modbus_read_input_registers", 1,modbusreadCallback);
	ros::Subscriber chatter_sub2 = n.subscribe("modbus_read_holding_registers", 1,modbusdataCallback);
  	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmdVelReceived);

  	ros::Publisher velocity_pub = n.advertise<std_msgs::Float32MultiArray>("/velocity", 1);
  	ros::Publisher voltage_pub = n.advertise<std_msgs::Float32>("/voltage", 1);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/map", 25);
	ros::Publisher robot_pose_pub  = n.advertise<modbustest::pose>("/robot_pose", 25);
  	ros::Publisher WriteMultipleRegisters_pub = n.advertise<modbustest::WriteMultipleRegisters>("modbus_writemultipleregisters", 1);

	ROS_INFO("Connected to rabbit base robot");
	ros::Rate r(100);

	while(n.ok())
	{
		signed long int ldiff = 0;
		signed long int rdiff = 0;
		current_time = ros::Time::now();

		//Setting robot to come rest if cmd_vel fails to come within the specified timeout in secs
		if(cmd_vel_time_out_disable == false) {
		current_wall_time = ros::WallTime::now();
		//ROS_INFO("Command velocity timeout %f :: %f : %f",current_wall_time,last_cmd_vel_received, cmd_vel_time_out);
		if((current_wall_time - last_cmd_vel_received).toSec() > cmd_vel_time_out)
		{
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
			ldiff = ml_dir * (abs(32768 - abs(last_left_encoder)) + abs(left_encoder));
			}
			else
			{
			ldiff = ml_dir * abs(abs(left_encoder) - abs(last_left_encoder));
			}
		last_left_encoder = left_encoder;
		}
		if(right_encoder!= last_right_encoder)
		{
		if((last_right_encoder > right_encoder)&&(right_encoder>0))		ROS_INFO("rdiff ");
			if((last_right_encoder > right_encoder)&&(last_right_encoder > 32000 )&&(right_encoder < 1400))
			{
			rdiff = mr_dir * (abs(32768 - abs(last_right_encoder)) + abs(right_encoder));
			}
			else
			{
			rdiff = mr_dir * abs(abs(right_encoder) - abs(last_right_encoder));
			}
		last_right_encoder = right_encoder;
		}

		double dist = ((rdiff + ldiff) * RABBIT_PULSES_TO_M)/2; 
		double ang = ((rdiff - ldiff) * RABBIT_PULSES_TO_M) / RABBIT_AXLE_LENGTH;
		if((abs(ldiff)>0)|(abs(rdiff)>0))
		ROS_INFO("diff %li : %li encoder L:%i:%lli R:%i:%lli D:%f A:%f ",ldiff, rdiff, ml_dir, left_encoder, mr_dir, right_encoder, (double)dist, (double)ang);
		
		// Update odometry
		odometry_yaw_ = NORMALIZE(odometry_yaw_ + ang);			// rad
		odometry_x_ = odometry_x_ + dist*cos(odometry_yaw_);		// m
		odometry_y_ = odometry_y_ + dist*sin(odometry_yaw_);		// m	

		dt = (current_time - last_time).toSec();
		last_time = current_time;
		vel_x = (odometry_x_ - last_x)/dt;
		vel_y = (odometry_y_ - last_y)/dt;
		vel_yaw = (odometry_yaw_ - last_yaw)/dt;

		last_x = odometry_x_;
		last_y = odometry_y_;
		last_yaw = odometry_yaw_;
		
		mvel.data.clear();
		if(ldiff > 0)		xldiff = ldiff;
		if(rdiff > 0)		xrdiff = rdiff;
		mvel.data.push_back(xldiff);
		mvel.data.push_back(xrdiff);

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		//geometry_msgs::Quaternion odom_quat;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_yaw_);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom_frame";
		odom_trans.child_frame_id = "base_link_frame";
		
		odom_trans.transform.translation.x = odometry_x_;
		odom_trans.transform.translation.y = odometry_y_;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom_frame";
		
		//set the position
		odom.pose.pose.position.x = odometry_x_;
		odom.pose.pose.position.y = odometry_y_;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		odom.child_frame_id = "base_link_frame";
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = vel_y;
		odom.twist.twist.angular.z = vel_yaw;

		//set the robot pose
		robot_pose.x = odometry_x_;
		robot_pose.y = odometry_y_;
		robot_pose.theta = odometry_yaw_;
		robot_pose.linear_velocity = vel_x;
		robot_pose.angular_velocity = vel_yaw;

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

		for(int i=0; i < motor_registers_count;i++)
		{
			long int data = motor_data[i];
			mb_motor_pub.write_multiple_registers.push_back(data);
			//ROS_INFO("(%i) Data: %ld", i, data);
		}

		//publish the odometer message
		odom_pub.publish(odom);
		velocity_pub.publish(mvel);

		//publish the robot velocity message		
		robot_pose_pub.publish(robot_pose);

		//publish the robot velocity message		
		WriteMultipleRegisters_pub.publish(mb_motor_pub);

		mb_voltage_pub.data = avoltage;
		voltage_pub.publish(mb_voltage_pub);

		ros::spinOnce();
		r.sleep();
	}


  return 0;
}

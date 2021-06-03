#include <ros/ros.h>
#include <modbustest/readinputregisters.h>
#include <modbustest/readholdingregisters.h>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <modbustest/pose.h>

//  RABBIT Dimensions
#define RABBIT_DIAMETER		0.100
// Distance between wheel of robot
#define RABBIT_AXLE_LENGTH	0.280
#define RABBIT_MAX_RPM		255

#define  RABBIT_MAX_LIN_VEL_MM_S	500
#define  RABBIT_MAX_ANG_VEL_RAD_S	20.94		//((2*pi)/60)*200 RPM = 20.94
//#define  RABBIT_MAX_RADIUS_MM		2000

//!  RABBIT max encoder counts
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

unsigned long long int encoder1_counts = 0;
unsigned long long int encoder2_counts = 0;

int ml_slaveid, mr_slaveid;
int ml_index, mr_index;

signed int ml_dir, mr_dir;

unsigned long long int left_encoder, right_encoder;
double odometry_x_, odometry_y_, odometry_yaw_;
double last_cmd_vel_received;

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
		left_encoder= encoder1_counts;
		//tmp1 = msg->read_input_registers[m1encoder_index-2];
		//tmp2 = msg->read_input_registers[m1encoder_index-1];
		//ROS_INFO("slave %i index:%i Data::%lli:%i:%i",slaveid, m1encoder_index, encoder1_counts,tmp1,tmp2);
	}
	else if(slaveid == m2encoder_slaveid)
	{
		encoder2_counts=msg->read_input_registers[m2encoder_index];
		right_encoder= encoder2_counts;
		//tmp1 = msg->read_input_registers[m2encoder_index-2];
		//tmp2 = msg->read_input_registers[m2encoder_index-1];
		//ROS_INFO("slave %i index:%i Data::%lli:%i:%i",slaveid, m2encoder_index, encoder2_counts,tmp1,tmp2);
	}
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
  ros::init(argc, argv, "odometer_node");
  double last_x, last_y, last_yaw;

  double vel_x, vel_y, vel_yaw;
  double dt;
  unsigned long long int last_left_encoder, last_right_encoder;

  last_x =0;
  last_y = 0;
  last_yaw = 0;
  odometry_x_ = 0;
  odometry_y_ = 0;
  odometry_yaw_ = 0;
  last_left_encoder = 0;
  last_right_encoder = 0;

  ros::NodeHandle n;
  std_msgs::Float32MultiArray mvel;

	//Getting slave details, coil counts and starting address register details from ros parameters
	n.param<int>("mencoder/m1encoder_slaveid", m1encoder_slaveid, 51);
	n.param<int>("mencoder/m2encoder_slaveid", m2encoder_slaveid, 53);

	n.param<int>("mencoder/m1encoder_index", m1encoder_index, 7);
	n.param<int>("mencoder/m2encoder_index", m2encoder_index, 7);

	n.param<int>("mmotor/motor_slaveid", ml_slaveid, 3);
	n.param<int>("mmotor/motor_slaveid", mr_slaveid, 3);

	n.param<int>("mmotor/ml_dirindex", ml_index, 1);
	n.param<int>("mmotor/mr_dirindex", mr_index, 4);


	if((m1encoder_slaveid == 0) && (m2encoder_slaveid == 0))
	{
	ROS_ERROR("Error: Parameters for encoder - slave id  not loaded sucessfully!");
	return 0;
	}

	modbustest::pose robot_pose;

	robot_pose.x = robot_pose.y = robot_pose.theta = 0;
	robot_pose.linear_velocity = robot_pose.angular_velocity = 0;

	tf::TransformBroadcaster odom_broadcaster;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/map", 25);
	ros::Publisher robot_pose_pub  = n.advertise<modbustest::pose>("/robot_pose", 25);
	ros::Subscriber chatter_sub1 = n.subscribe("modbus_read_input_registers", 1,modbusreadCallback);
	ros::Subscriber chatter_sub2 = n.subscribe("modbus_read_holding_registers", 1,modbusdataCallback);
  	ros::Publisher velocity_pub = n.advertise<std_msgs::Float32MultiArray>("velocity", 1);

	ROS_INFO("Connected to rabbit base robot");

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate r(100);

	signed long int xldiff = 0;
	signed long int xrdiff = 0;

	while(n.ok())
	{
		signed long int ldiff = 0;
		signed long int rdiff = 0;
		current_time = ros::Time::now();
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
		odom_trans.header.frame_id = "map";
		odom_trans.child_frame_id = "base_link";
		
		odom_trans.transform.translation.x = odometry_x_;
		odom_trans.transform.translation.y = odometry_y_;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "map";
		
		//set the position
		odom.pose.pose.position.x = odometry_x_;
		odom.pose.pose.position.y = odometry_y_;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = vel_y;
		odom.twist.twist.angular.z = vel_yaw;

		//set the robot pose
		robot_pose.x = odometry_x_;
		robot_pose.y = odometry_y_;
		robot_pose.theta = odometry_yaw_;
		robot_pose.linear_velocity = vel_x;
		robot_pose.angular_velocity = vel_yaw;

		//publish the odometer message
		odom_pub.publish(odom);
		velocity_pub.publish(mvel);

		//publish the robot velocity message		
		robot_pose_pub.publish(robot_pose);
		ros::spinOnce();
		r.sleep();
	}


  return 0;
}

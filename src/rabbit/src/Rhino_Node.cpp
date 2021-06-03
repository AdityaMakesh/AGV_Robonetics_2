#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <rabbit/ServoMotor.h>
#include <string>

#include <rabbit/autoCalibrate.h>
#include <rabbit/getAbsolutePostion.h>
#include <rabbit/getFeedbackGain.h>
#include <rabbit/getIntegralGain.h>
#include <rabbit/getMaxMotorSpeed.h>
#include <rabbit/getPositionEncoder.h>
#include <rabbit/getProportionateGain.h>
#include <rabbit/loadFactorySettings.h>
#include <rabbit/readDamping.h>
#include <rabbit/readMotorSpeed.h>
#include <rabbit/setAbsolutePostion.h>
#include <rabbit/setFeedbackGain.h>
#include <rabbit/setIntegralGain.h>
#include <rabbit/setMaxMotorSpeed.h>
#include <rabbit/setMotorSpeed.h>
#include <rabbit/setPositionEncoder.h>
#include <rabbit/setProportionateGain.h>
#include <rabbit/setRelativePostion.h>
#include <rabbit/writeDamping.h>
#include <rabbit/pose.h>

//cmoomand velocity timeout in secs
#define CMDVEL_TIMEOUT			5

// Positions
#define LEFT				0
#define RIGHT				1

//  RABBIT Dimensions
#define RABBIT_BUMPER_X_OFFSET		0.001
#define RABBIT_DIAMETER		0.090
#define RABBIT_AXLE_LENGTH		0.290

#define  RABBIT_MAX_LIN_VEL_MM_S	500
#define  RABBIT_MAX_ANG_VEL_RAD_S	20.94		//((2*pi)/60)*200 RPM = 20.94
//#define  RABBIT_MAX_RADIUS_MM		2000

//!  RABBIT max encoder counts
#define  RABBIT_MAX_ENCODER_COUNTS	1800
//!  RABBIT encoder pulses to meter constant
//Wheel Diameter 96mm
#define  RABBIT_PULSES_TO_M		0.00016755			//(2 * PI * Radius of wheel)/Encoder counts per revolution

#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

//! Delta encoder counts.
long long int encoder_counts_[2];

std::string port_a;
std::string port_b;
ServoMotor::RhinoInterface * rabbit_left;
ServoMotor::RhinoInterface * rabbit_right;
double odometry_x_, odometry_y_, odometry_yaw_;
double last_cmd_vel_received;

std::string prefixTopic(std::string prefix, char * name)
{
	std::string topic_name = prefix;
	topic_name.append(name);
	
	return topic_name;
}

bool autoCalibrate(rabbit::autoCalibrate::Request  &req, rabbit::autoCalibrate::Response &res)
{
  rabbit_left->autoCalibrate();
  ros::Duration(5.0).sleep();
  rabbit_right->autoCalibrate();
  ros::Duration(5.0).sleep();
  ROS_INFO("response: Auto Calibartion Finished");
  return true;
}

bool getAbsolutePostion(rabbit::getAbsolutePostion::Request  &req, rabbit::getAbsolutePostion::Response &res)
{
  res.left_position = rabbit_left->getAbsolutePostion();
  res.right_position = rabbit_right->getAbsolutePostion();
  ROS_INFO("response: Data %li, %li",res.left_position,res.right_position);
  return true;
}

bool getFeedbackGain(rabbit::getFeedbackGain::Request  &req, rabbit::getFeedbackGain::Response &res)
{
  res.left_gain = rabbit_left->getFeedbackGain();
  res.right_gain = rabbit_right->getFeedbackGain();
  ROS_INFO("response: Data %i, %i",res.left_gain,res.right_gain);
  return true;
}	
bool getIntegralGain(rabbit::getIntegralGain::Request  &req, rabbit::getIntegralGain::Response &res)
{
  res.left_gain = rabbit_left->getIntegralGain();
  res.right_gain = rabbit_right->getIntegralGain();
  ROS_INFO("response: Data %i, %i",res.left_gain,res.right_gain);
  return true;
}

bool getMaxMotorSpeed(rabbit::getMaxMotorSpeed::Request  &req, rabbit::getMaxMotorSpeed::Response &res)
{
  res.left_speed = rabbit_left->getMaxMotorSpeed();
  res.right_speed = rabbit_right->getMaxMotorSpeed();
  ROS_INFO("response: Data %li, %li",res.left_speed,res.right_speed);
  return true;
}

bool getPositionEncoder(rabbit::getPositionEncoder::Request  &req, rabbit::getPositionEncoder::Response &res)
{
  res.left_encoder = rabbit_left->getPositionEncoder();
  res.right_encoder = rabbit_right->getPositionEncoder();
  ROS_INFO("response: Data %li, %li",res.left_encoder,res.right_encoder);
  return true;
}	
bool getProportionateGain(rabbit::getProportionateGain::Request  &req, rabbit::getProportionateGain::Response &res)
{
  res.left_gain = rabbit_left->getProportionateGain();
  res.right_gain = rabbit_right->getProportionateGain();
  ROS_INFO("response: Data %i, %i",res.left_gain,res.right_gain);
  return true;
}	
	
bool loadFactorySettings(rabbit::loadFactorySettings::Request  &req, rabbit::loadFactorySettings::Response &res)
{
  rabbit_left->loadFactorySettings();
  ros::Duration(2.0).sleep();
  rabbit_right->loadFactorySettings();
  ros::Duration(2.0).sleep();
  ROS_INFO("response: Factory Settings Restore Finished");
  return true;
}
bool readDamping(rabbit::readDamping::Request  &req, rabbit::readDamping::Response &res)
{
  res.left_damp = rabbit_left->readDamping();
  res.right_damp = rabbit_right->readDamping();
  ROS_INFO("response: Data %li, %li",res.left_damp,res.right_damp);
  return true;
}
bool readMotorSpeed(rabbit::readMotorSpeed::Request  &req, rabbit::readMotorSpeed::Response &res)
{
  res.left_speed = rabbit_left->readMotorSpeed();
  res.right_speed = rabbit_right->readMotorSpeed();
  ROS_INFO("response: Data %li, %li",res.left_speed,res.right_speed);
  return true;
}	
bool setAbsolutePostion(rabbit::setAbsolutePostion::Request  &req, rabbit::setAbsolutePostion::Response &res)
{
  bool rdata = false;
  res.result = -1;
  if((rabbit_left->setAbsolutePostion(req.left_position) == 0) && (rabbit_right->setAbsolutePostion(req.right_position))==0)
  {
  rdata = true;
  res.result = 0;
  ROS_INFO("request: Data %li, %li",req.left_position,req.right_position);
  }
  return rdata;
}	
bool setFeedbackGain(rabbit::setFeedbackGain::Request  &req, rabbit::setFeedbackGain::Response &res)
{
  bool rdata = false;
  res.result = -1;
  if((rabbit_left->setFeedbackGain(req.left_gain) == 0) && (rabbit_right->setFeedbackGain(req.right_gain))==0)
  {
  rdata = true;
  res.result = 0;
  ROS_INFO("request: Data %li, %li",req.left_gain,req.right_gain);
  }
  return rdata;
}	
bool setIntegralGain(rabbit::setIntegralGain::Request  &req, rabbit::setIntegralGain::Response &res)
{
  bool rdata = false;
  res.result = -1;
  if((rabbit_left->setIntegralGain(req.left_igain) == 0) && (rabbit_right->setIntegralGain(req.right_igain))==0)
  {
  rdata = true;
  res.result = 0;
  ROS_INFO("request: Data %li, %li",req.left_igain,req.right_igain);
  }
  return rdata;
}	
bool setMaxMotorSpeed(rabbit::setMaxMotorSpeed::Request  &req, rabbit::setMaxMotorSpeed::Response &res)
{
  bool rdata = false;
  res.result = -1;
  if((rabbit_left->setMaxMotorSpeed(req.left_speed) == 0) && (rabbit_right->setMaxMotorSpeed(req.right_speed))==0)
  {
  rdata = true;
  res.result = 0;
  ROS_INFO("request: Data %li, %li",req.left_speed,req.right_speed);
  }
  return rdata;
}
bool setMotorSpeed(rabbit::setMotorSpeed::Request  &req, rabbit::setMotorSpeed::Response &res)
{
  bool rdata = false;
  res.result = -1;
  if((rabbit_left->writeMotorSpeed(req.left_speed) == 0) && (rabbit_right->writeMotorSpeed(req.right_speed))==0)
  {
  rdata = true;
  res.result = 0;
  ROS_INFO("request: Data %li, %li",req.left_speed,req.right_speed);
  }
  return rdata;
}	
bool setPositionEncoder(rabbit::setPositionEncoder::Request  &req, rabbit::setPositionEncoder::Response &res)
{
  bool rdata = false;
  res.result = -1;
  if((rabbit_left->setPositionEncoder(req.left_encoder) == 0) && (rabbit_right->setPositionEncoder(req.right_encoder))==0)
  {
  rdata = true;
  res.result = 0;
  ROS_INFO("request: Data %li, %li",req.left_encoder,req.right_encoder);
  }
  return rdata;
}	
bool setProportionateGain(rabbit::setProportionateGain::Request  &req, rabbit::setProportionateGain::Response &res)
{
  bool rdata = false;
  res.result = -1;
  if((rabbit_left->setIntegralGain(req.left_pgain) == 0) && (rabbit_right->setIntegralGain(req.right_pgain))==0)
  {
  rdata = true;
  res.result = 0;
  ROS_INFO("request: Data %li, %li",req.left_pgain,req.right_pgain);
  }
  return rdata;
}	
bool setRelativePostion(rabbit::setRelativePostion::Request  &req, rabbit::setRelativePostion::Response &res)
{
  bool rdata = false;
  res.result = -1;
  if((rabbit_left->setRelativePostion(req.left_position) == 0) && (rabbit_right->setRelativePostion(req.right_position))==0)
  {
  rdata = true;
  res.result = 0;
  ROS_INFO("request: Data %li, %li",req.left_position,req.right_position);
  }
  return rdata;
}
bool writeDamping(rabbit::writeDamping::Request  &req, rabbit::writeDamping::Response &res)
{
  bool rdata = false;
  res.result = -1;
  if((rabbit_left->writeDamping(req.left_damping) == 0) && (rabbit_right->writeDamping(req.right_damping))==0)
  {
  rdata = true;
  res.result = 0;
  ROS_INFO("request: Data %li, %li",req.left_damping,req.right_damping);
  }
  return rdata;
}

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
//int left_speed_mm_s = (int)((cmd_vel->linear.x-RABBIT_AXLE_LENGTH*cmd_vel->angular.z/2)*1e3);	// Left wheel velocity in mm/s
//int right_speed_mm_s = (int)((cmd_vel->linear.x+RABBIT_AXLE_LENGTH*cmd_vel->angular.z/2)*1e3);	// Right wheel velocity in mm/s

int left_speed_rad_s = (int)((2*cmd_vel->linear.x-RABBIT_AXLE_LENGTH*cmd_vel->angular.z)/(RABBIT_AXLE_LENGTH*RABBIT_DIAMETER));	// Left wheel velocity in rad/s
int right_speed_rad_s = (int)((2*cmd_vel->linear.x+RABBIT_AXLE_LENGTH*cmd_vel->angular.z)/(RABBIT_AXLE_LENGTH*RABBIT_DIAMETER));	// Right wheel velocity in rad/s

int left_speed_rps = abs(3.14/left_speed_rad_s );
int right_speed_rps = abs(3.14/right_speed_rad_s );

if(rabbit_left->writeMotorSpeed(left_speed_rps*-1) == 0) {ROS_INFO("Left wheel velocity:%i rps",left_speed_rps);}
if(rabbit_right->writeMotorSpeed(right_speed_rps) == 0) {ROS_INFO("Right wheel velocity:%i rps",right_speed_rps);}

if(rabbit_left->writeMotorSpeed(left_speed_rad_s*-1) == 0) {ROS_INFO("Right wheel velocity:%i",left_speed_rad_s);}
if(rabbit_right->writeMotorSpeed(right_speed_rad_s) == 0) {ROS_INFO("Left wheel velocity:%i",right_speed_rad_s);}

last_cmd_vel_received = ros::WallTime::now().toSec();
}

// Calculate Robot odometry
void calculateOdometry()
{	
	double dist = (encoder_counts_[RIGHT]*RABBIT_PULSES_TO_M + (encoder_counts_[LEFT]*RABBIT_PULSES_TO_M)*-1) / 2.0; 
	double ang = (encoder_counts_[RIGHT]*RABBIT_PULSES_TO_M - (encoder_counts_[LEFT]*RABBIT_PULSES_TO_M)*-1) / RABBIT_AXLE_LENGTH;

	// Update odometry
	odometry_yaw_ = NORMALIZE(odometry_yaw_ + ang);			// rad
	odometry_x_ = odometry_x_ + dist*cos(odometry_yaw_);		// m
	odometry_y_ = odometry_y_ + dist*sin(odometry_yaw_);		// m
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rabbit_light_node");

	ROS_INFO("Rabbit for ROS %.2f", NODE_VERSION);
	
	double last_x, last_y, last_yaw;
	
	double vel_x, vel_y, vel_yaw;
	double dt;

	long long int left_encoder, right_encoder;
	long long int last_left_encoder, last_right_encoder;

	last_x =0;
	last_y = 0;
	last_yaw = 0;
	odometry_x_ = 0;
	odometry_y_ = 0;
	odometry_yaw_ = 0;
	last_left_encoder = 0;
	last_right_encoder = 0;
	ros::NodeHandle n;

	n.getParam("rnode/rabbit/port_a", port_a);
	n.getParam("rnode/rabbit/port_b", port_b);

//end of modification
		
	rabbit_left = new ServoMotor::RhinoInterface(port_a.c_str());
	rabbit_right = new ServoMotor::RhinoInterface(port_b.c_str());
	tf::TransformBroadcaster odom_broadcaster;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/map", 50);
	ros::Publisher robot_pose_pub  = n.advertise<rabbit::pose>("/robot_pose", 50);
	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);

	ros::ServiceServer service_autoCalibrate = n.advertiseService("autoCalibrate", autoCalibrate);
	ros::ServiceServer service_getAbsolutePostion = n.advertiseService("getAbsolutePostion", getAbsolutePostion);
	ros::ServiceServer service_getFeedbackGain = n.advertiseService("getFeedbackGain", getFeedbackGain);
	ros::ServiceServer service_getIntegralGain = n.advertiseService("getIntegralGain", getIntegralGain);
	ros::ServiceServer service_getMaxMotorSpeed = n.advertiseService("getMaxMotorSpeed", getMaxMotorSpeed);
	ros::ServiceServer service_getPositionEncoder = n.advertiseService("getPositionEncoder", getPositionEncoder);
	ros::ServiceServer service_getProportionateGain = n.advertiseService("getProportionateGain", getProportionateGain);
	ros::ServiceServer service_loadFactorySettings = n.advertiseService("loadFactorySettings", loadFactorySettings);
	ros::ServiceServer service_readDamping = n.advertiseService("readDamping", readDamping);
	ros::ServiceServer service_readMotorSpeed = n.advertiseService("readMotorSpeed", readMotorSpeed);
	ros::ServiceServer service_setAbsolutePostion = n.advertiseService("setAbsolutePostion", setAbsolutePostion);
	ros::ServiceServer service_setFeedbackGain = n.advertiseService("setFeedbackGain", setFeedbackGain);
	ros::ServiceServer service_setIntegralGain = n.advertiseService("setIntegralGain", setIntegralGain);
	ros::ServiceServer service_setMaxMotorSpeed = n.advertiseService("setMaxMotorSpeed", setMaxMotorSpeed);
	ros::ServiceServer service_setMotorSpeed = n.advertiseService("setMotorSpeed", setMotorSpeed);
	ros::ServiceServer service_setPositionEncoder = n.advertiseService("setPositionEncoder", setPositionEncoder);
	ros::ServiceServer service_setProportionateGain = n.advertiseService("setProportionateGain", setProportionateGain);
	ros::ServiceServer service_setRelativePostion = n.advertiseService("setRelativePostion", setRelativePostion);
	ros::ServiceServer service_writeDamping = n.advertiseService("writeDamping", writeDamping);

	if(( rabbit_left->openSerialPort() == 0)&&( rabbit_right->openSerialPort() == 0))
		ROS_INFO("Connected to eyantra base robot with MA:%s and MB:%s",port_a.c_str(),port_b.c_str());
	else
	{
		ROS_FATAL("Could not connect to eyantra base robot.");
		ROS_BREAK();
	}
	
	ros::Duration(0.5).sleep(); // sleep for half a second
	left_encoder = rabbit_left->getPositionEncoder();
	right_encoder = rabbit_right->getPositionEncoder();
	ROS_INFO("starting EA:%lli and EB:%lli", left_encoder, right_encoder);
/*
	while((left_encoder != 0)&&(right_encoder != 0)&&ros::ok())
	{
		rabbit_left->setPositionEncoder(0);
		rabbit_right->setPositionEncoder(0);
		ros::Duration(3).sleep(); // sleep for half a second
		
		for(int k=0;k<20;k++)
		{
		left_encoder = rabbit_left->getPositionEncoder();
		right_encoder = rabbit_right->getPositionEncoder();
		}
		ROS_INFO("clear EA:%lli and EB:%lli", left_encoder, right_encoder);
		//ROS_INFO("Clearing Encoder values and current value Left:%lli and Right:%lli",left_encoder,right_encoder);
	}
*/
	rabbit::pose robot_pose;

	robot_pose.x = robot_pose.y = robot_pose.theta = 0;
	robot_pose.linear_velocity = robot_pose.angular_velocity = 0;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate r(20.0);

	while(n.ok())
	{
		//ROS_INFO("Hai");		
		left_encoder = rabbit_left->getPositionEncoder();
		right_encoder = rabbit_right->getPositionEncoder();

		ROS_INFO("EA:%lli and EB:%lli last_X:%f and last_Y:%f",left_encoder, right_encoder, last_x, last_y);

		if((left_encoder!=-1)&&(right_encoder!=-1))
		{
		current_time = ros::Time::now();

		left_encoder = left_encoder * (-1);
		long long int ldiff = left_encoder - last_left_encoder;
		long long int rdiff = right_encoder - last_right_encoder;

		double dist = ((rdiff * RABBIT_PULSES_TO_M) + (ldiff * RABBIT_PULSES_TO_M))/2; 
		double ang = ((rdiff*RABBIT_PULSES_TO_M) - (ldiff*RABBIT_PULSES_TO_M)) / RABBIT_AXLE_LENGTH;
		last_left_encoder = left_encoder;
		last_right_encoder = right_encoder;

		// Update odometry
		odometry_yaw_ = NORMALIZE(odometry_yaw_ + ang);			// rad
		odometry_x_ = odometry_x_ + dist*cos(odometry_yaw_);		// m
		odometry_y_ = odometry_y_ + dist*sin(odometry_yaw_);		// m	

		ROS_INFO("E:%lli:%lli X:%f:%f:%f Dist:%f and Angle:%f",left_encoder, right_encoder, odometry_x_, odometry_y_, odometry_yaw_, dist, ang);

		dt = (current_time - last_time).toSec();
		last_time = current_time;
		vel_x = (odometry_x_ - last_x)/dt;
		vel_y = (odometry_y_ - last_y)/dt;
		vel_yaw = (odometry_yaw_ - last_yaw)/dt;

		last_x = odometry_x_;
		last_y = odometry_y_;
		last_yaw = odometry_yaw_;
		}
		
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


		//Setting robot to come rest if cmd_vel fails to come within the specified timeout in secs
		if((ros::WallTime::now().toSec() - last_cmd_vel_received) > CMDVEL_TIMEOUT)
		{
		rabbit_left->writeMotorSpeed(0);
		rabbit_right->writeMotorSpeed(0);
		ROS_INFO("Command velocity timeout");
		}

		//publish the odometer message
		odom_pub.publish(odom);

		//publish the robot velocity message		
		robot_pose_pub.publish(robot_pose);

		ros::spinOnce();
		r.sleep();
	}
	
	//rabbit->powerDown();
	rabbit_left->closeSerialPort();
	rabbit_right->closeSerialPort();
}

// EOF

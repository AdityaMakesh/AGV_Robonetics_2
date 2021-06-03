#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rabbit/pose.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define RABBIT_DIAMETER		0.0090

double vx, vy, vz, wx, wy, wz;
double px, py, pz, roll, pitch, yaw;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void poseCallback(const rabbit::pose::ConstPtr& msg)
{
px = msg->x;
py = msg->y;
pz = 0;
roll = pitch = 0;
yaw = msg->theta;
vx = msg->linear_velocity;
wz = msg->angular_velocity;

ROS_INFO("Pose: [P:%f: %f: %f  A:%f : %f : %f ]", px, py, pz, roll, pitch, yaw);
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

	unsigned int current_step=0;
	unsigned int total_steps=0;

	float kpw,kpx,kpy,kdw,kdx,kdy,wd,xd,yd;

	float Mz, Fx, f1, f2, u;

	ros::init(argc, argv, "rabbit_sqaure");
	ros::NodeHandle n;

	ros::Subscriber pose_sub = n.subscribe("robot_pose", 1000, poseCallback);
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

//	ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 1000, poseCallback);
//	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

	geometry_msgs::Twist base_cmd;

	ros::Time current_time, last_time, start_time;

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate loop_rate(5);

	n.getParam("control/kpx", kpx);
	n.getParam("control/kpy", kpy);
	n.getParam("control/kpw", kpw);
	n.getParam("control/kdx", kdx);
	n.getParam("control/kdy", kdy);
	n.getParam("control/kdw", kdw);

	n.getParam("control/wd", wd);
	n.getParam("control/xd", xd);
	n.getParam("control/yd", yd);

	std::vector<double> target_lx_list;
	std::vector<double> target_ly_list;
	std::vector<double> target_lz_list;
	std::vector<double> target_ax_list;
	std::vector<double> target_ay_list;
	std::vector<double> target_az_list;
	std::vector<double> target_time;

	n.getParam("target_lx", target_lx_list);
	n.getParam("target_ly", target_ly_list);
	n.getParam("target_lz", target_lz_list);
	n.getParam("target_ax", target_ax_list);
	n.getParam("target_ay", target_ay_list);
	n.getParam("target_az", target_az_list);
	n.getParam("target_t", target_time);

	
	total_steps = target_time.size();

	if(total_steps <= 0) 
	{
	ROS_ERROR("Error: Parameters not loaded sucessfully!");
	return 0;
	}
	else
	{
	ROS_INFO("param list size::%i",(int)target_time.size());
	}

	base_cmd. linear.x = base_cmd.linear.y = base_cmd.linear.z = 0;
	base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0; 

	ROS_INFO("Control constants: [kpx:%f kpy: %f kpw: %f  kdx:%f kdy: %f kdw: %f ]", kpx, kpy, kpw, kdx, kdy, kdw);
	ROS_INFO("Waiting 5 secs for the robot to get intialize");
	ros::Duration(5.0).sleep();
	ROS_INFO("Control loop begins now with %i steps",total_steps);

	start_time = ros::Time::now();
	xd = target_lx_list[current_step];
	yd = target_ly_list[current_step];
	wd = target_az_list[current_step];
	ROS_INFO("Control constants: [xd:%f  yd:%f wd:%f t: %i]", xd, yd, wd, current_step);

	while (ros::ok())
	{
	/**
	* This is a message object. You stuff it with data, and then publish it.
	*/
	current_time = ros::Time::now();
	if(current_step < total_steps)
	{

		//PID Control starts here
		u = sqrt(vx*vx+vy*vy);

		wd = atan((yd-py)/(xd-px));

		Fx = (kpx * cos(yaw) * (xd-px) + kpy * sin(yaw) * (yd-py) - kdx * u);
		Mz = (kpw * (wd - yaw) - kdw * wz);

		f1 = ((Mz + Fx * (RABBIT_DIAMETER/2))/RABBIT_DIAMETER);
		f2 = Fx- f1;

		base_cmd.linear.x = f1 + f2;
		base_cmd.linear.y = base_cmd.linear.z = 0;
		base_cmd.angular.x = base_cmd.angular.y = 0;
		base_cmd.angular.z = (f1-f2)*(RABBIT_DIAMETER/2);

		if((current_time.toSec() - start_time.toSec()) > target_time[current_step] )
		{
			current_step++;
			xd = target_lx_list[current_step];
			yd = target_ly_list[current_step];
			wd = target_az_list[current_step];

			start_time = current_time;
			ROS_INFO("Control constants: [xd:%f  yd:%f wd:%f t: %i]", xd, yd, wd, current_step);
		}
	}
	else if(current_step == total_steps)
	{
		base_cmd. linear.x = base_cmd.linear.y = base_cmd.linear.z = 0;
		base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0; 
		current_step++;
	}
	else
	{
		ROS_INFO("Task Completed");
		return 0;
	}

	ROS_INFO("ROBOT Pos: [L:%f: %f A:%f wd: %f f1:%f f2:%f]", px, py, yaw, wd, f1, f2);
	cmd_vel_pub.publish(base_cmd);

	ros::spinOnce();

	loop_rate.sleep();
	}


return 0;
}

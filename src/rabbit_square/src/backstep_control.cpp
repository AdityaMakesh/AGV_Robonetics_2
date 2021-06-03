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

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "simple_controller");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
//  ros::Subscriber vel_sub = n.subscribe("robot_vel", 1000, velocityCallback);
//  ros::Subscriber pose_sub = n.subscribe("robot_pose", 1000, poseCallback);
//  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Subscriber pose_sub = n.subscribe("robot_pose", 1000, poseCallback);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

//  ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 1000, poseCallback);
//  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

  geometry_msgs::Twist base_cmd;

float kpw,kpx,kpy,kdw,kdx,kdy,wd,xd,yd;

float Mz, Fx, f1, f2, u;

n.getParam("control/kpx", kpx);
n.getParam("control/kpy", kpy);
n.getParam("control/kpw", kpw);
n.getParam("control/kdx", kdx);
n.getParam("control/kdy", kdy);
n.getParam("control/kdw", kdw);

n.getParam("control/wd", wd);
n.getParam("control/xd", xd);
n.getParam("control/yd", yd);

ros::Rate loop_rate(5);

ROS_INFO("Control constants: [kpx:%f kpy: %f kpw: %f  kdx:%f kdy: %f kdw: %f xd:%f  yd:%f wd:%f ]", kpx, kpy, kpw, kdx, kdy, kdw, xd, yd, wd);

ROS_INFO("Waiting 5 secs for the robot to get intialize");
ros::Duration(5.0).sleep();
ROS_INFO("Control loop begins now");

base_cmd.linear.x = base_cmd.linear.y = base_cmd.linear.z = 0;
base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0; 

while(ros::ok())
{
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
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

  ROS_INFO("ROBOT Vel: [L:%f: %f: %f  A:%f : %f : %f ] C Vel: [L:%f : %f :%f A: %f : %f :%f]", vx, vy, vz, wx, wy,wz, base_cmd.linear.x, base_cmd.linear.y, base_cmd.linear.z, base_cmd.angular.x, base_cmd.angular.y, base_cmd.angular.z);
  ROS_INFO("ROBOT Pos: [L:%f: %f: %f  A:%f : %f : %f ]", px, py, pz, roll, pitch, yaw);

//base_cmd.linear.x = base_cmd.linear.y = base_cmd.linear.z = 0;
//base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0; 

  cmd_vel_pub.publish(base_cmd);

  ros::spinOnce();
  loop_rate.sleep();
}
  return 0;
}

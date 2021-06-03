#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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
  ros::init(argc, argv, "rabbit_sqaure");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
//  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Rate loop_rate(1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  geometry_msgs::Twist base_cmd;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  float distance = 0.5;
  float angle = 1.5708;
  float speed = 0.2;
  float angular_speed = 0.1;
  float current_distance = 0.0;
  float current_angle = 0.0;
  int segment=0;
  int count=0;

  base_cmd. linear.x = base_cmd.linear.y = base_cmd.linear.z = 0;
  base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0; 
  float start_time = ros::Time::now().toSec();
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    current_time = ros::Time::now();
    float t = current_time.toSec() - start_time; 
//    base_cmd.linear.x = abs(1*cos(t));
//    base_cmd.linear.y = abs(1*sin(t));
//    base_cmd.linear.z = 0;

    base_cmd.linear.x = 1;
    base_cmd.linear.y = 0;
    base_cmd.linear.z = 0;

    base_cmd.angular.x = base_cmd.angular.y = 0;
    base_cmd.angular.z = 1;

    ROS_INFO("T:: %f Linear X::%f Y::%f Z::%f Angular X::%f Y::%f Z::%f , %f : %f", t, base_cmd.linear.x,base_cmd.linear.y,base_cmd.linear.z, base_cmd.angular.x, base_cmd.angular.y,base_cmd.angular.z, cos(5*t)*(180.0/3.14), sin(5*t)*(180.0/3.14));

    cmd_vel_pub.publish(base_cmd);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

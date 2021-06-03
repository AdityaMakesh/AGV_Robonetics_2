#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
#define FALSE 0
#define TRUE 1

ros::Time last_measured_time, prev_measured_time;
float av_x, av_y, av_z;
float la_x, la_y, la_z;

#define CALIBRATE_SAMPLE_SIZE 1024
#define SAMPLE_WINDOW_SIZE 64

void filter_imu_data(const sensor_msgs::Imu::ConstPtr& msg)
{
  last_measured_time = msg->header.stamp;
  av_x = msg->angular_velocity.x; av_y = msg->angular_velocity.y; av_z = msg->angular_velocity.z;
  la_x = msg->linear_acceleration.x; la_y = msg->linear_acceleration.y; la_z = msg->linear_acceleration.z;
  ROS_INFO("Angular: [%f, %f, %f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  ROS_INFO("Linear: [%f, %f, %f]", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

int main(int argc, char **argv)
{
  ros::Time last_sample_time, current_sample_time;
  float bav_x, bav_y, bav_z;
  float bla_x, bla_y, bla_z;

  float cav_x, cav_y, cav_z;
  float cla_x, cla_y, cla_z;

  float ssav_x, ssav_y, ssav_z;
  float ssla_x, ssla_y, ssla_z;

  float pla_x, pla_y, pla_z;
  float pav_x, pav_y, pav_z;

  float plv_x, plv_y, plv_z;
  float clv_x, clv_y, clv_z;

  float pap_x, pap_y, pap_z;
  float cap_x, cap_y, cap_z;

  float plp_x, plp_y, plp_z;
  float clp_x, clp_y, clp_z;

  int calibated;
  int count;

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
  ros::init(argc, argv, "imu_filter");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("imu/data_raw", 1000, filter_imu_data);
  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu/data", 1000);

  ros::Rate loop_rate(20);

  pla_x = pla_y = pla_z = 0;
  pav_x = pav_y = pav_z = 0;

  cav_x = cav_y = cav_z = 0;
  cla_x = cla_y = cla_z = 0;

  plv_x = plv_y = plv_z = 0;
  clv_x = clv_y = clv_z = 0;

  pap_x = pap_y = pap_z = 0;
  cap_x = cap_y = cap_z = 0;

  plp_x = plp_y = plp_z = 0;
  clp_x = clp_y = clp_z = 0;

  calibated = 0;
  count = 0;

   while (ros::ok())
  {
    sensor_msgs::Imu msg1;
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

     if(calibated == TRUE)
     {
       //Adding up samples to get cummulated data
       if(last_measured_time.toSec() > prev_measured_time.toSec())
       {
         cav_x = cav_x + av_x;
         cav_y = cav_y + av_y;
         cav_z = cav_z + av_z;

         cla_x = cla_x + la_x;
         cla_y = cla_y + la_y;
         cla_z = cla_z + la_z;

         count++;
         prev_measured_time = last_measured_time;
       }
       //Averaging of sample after getting sufficient samples
       if(count >= SAMPLE_WINDOW_SIZE)
       {
         //gaussian window samples
         ssav_x = cav_x / count - bav_x;
         ssav_y = cav_y / count - bav_y;
         ssav_z = cav_z / count - bav_z;

         ssla_x = cla_x / count - bla_x;
         ssla_y = cla_y / count - bla_y;
         ssla_z = cla_z / count - bla_z;

         current_sample_time = ros::Time::now();
         float sample_time_interval = (current_sample_time.toSec() - last_sample_time.toSec());

         //First inetgration to get angular angles
         cap_x = pap_x + pav_x + (ssav_x - pav_x)/sample_time_interval;
         cap_y = pap_y + pav_y + (ssav_y - pav_y)/sample_time_interval;
         cap_z = pap_z + pav_z + (ssav_z - pav_z)/sample_time_interval;

         //First integration to get linear velocity
         clv_x = plv_x + pla_x + (sslax - pla_x)/sample_time_interval;
         clv_y = plv_y + pla_y + (sslay - pla_y)/sample_time_interval;
         clv_z = plv_z + pla_z + (sslaz - pla_z)/sample_time_interval;

         //First integration to get linear position
         clp_x = plp_x + plv_x + (clv_x - plv_x)/sample_time_interval;
         clp_y = plp_y + plv_y + (clv_y - plv_y)/sample_time_interval;
         clp_z = plp_z + plv_z + (clv_z - plv_z)/sample_time_interval;

         last_sample_time = current_sample_time;
         count = 0;

         msg1.header.stamp = current_sample_time;
         msg1.header.frame_id = '0';  // no frame

         // Read gyroscope values.
         // At default sensitivity of 250deg/s we need to scale by 131.
         msg1.angular_velocity.x = cav_x;
         msg1.angular_velocity.y = cav_y;
         msg1.angular_velocity.z = cav_z;

         // Read accelerometer values.
         // At default sensitivity of 2g we need to scale by 16384.
         // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
         // But! Imu msg docs say acceleration should be in m/2 so need to *9.807
         msg1.linear_acceleration.x = cla_x;
         msg1.linear_acceleration.y = cla_y;
         msg1.linear_acceleration.z = cla_z;
       }


     }
     else
     {
       //Adding up samples to get cummulated data
       if(last_measured_time.toSec() > prev_measured_time.toSec())
       {
         cav_x = cav_x + av_x;
         cav_y = cav_y + av_y;
         cav_z = cav_z + av_z;

         cla_x = cla_x + la_x;
         cla_y = cla_y + la_y;
         cla_z = cla_z + la_z;

         count++
         prev_measured_time = last_measured_time;
       }
       //Averaging of sample after getting sufficient samples
       if(count >= CALIBRATE_SAMPLE_SIZE)
       {
         bav_x = cav_x / count;
         bav_y = cav_y / count;
         bav_z = cav_z / count;

         bla_x = cla_x / count;
         bla_y = cla_y / count;
         bla_z = cla_z / count;

         calibated = TRUE;
         count = 0;
         cav_x = cav_y = cav_z = 0;
         cla_x = cla_y = cla_z = 0;
       }
     }

    pub.publish(msg1);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

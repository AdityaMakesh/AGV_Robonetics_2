/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <rabbit_imu/AllDevices.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace cacaosd_i2cport;
using namespace cacaosd_mpu6050;
using namespace cacaosd_hmc5883l;

#define FALSE 0
#define TRUE 1

#define CALIBRATE_SAMPLE_SIZE 1024
#define SAMPLE_WINDOW_SIZE 64
#define SAMPLE_INTERVAL_TIME 0.02

double av_rescale;
double la_rescale;

float av_x, av_y, av_z;
float la_x, la_y, la_z;

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

int ctrl;

double imu_yaw;

MPU6050 *mpu6050;

void filter_imu_data(const ros::TimerEvent& eventTime)
{
	// Read gyroscope values.
	// At default sensitivity of 250deg/s we need to scale by 131.
	av_x = (float) mpu6050->getAccelerationZ() / av_rescale;
	av_y = (float) mpu6050->getAngularVelocityY() / av_rescale;
	av_z = (float) mpu6050->getAngularVelocityZ() / av_rescale;

	// Read accelerometer values.
	// At default sensitivity of 2g we need to scale by 16384.
	// Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
	// But! Imu msg docs say acceleration should be in m/2 so need to *9.807
	la_x = (float) mpu6050->getAccelerationX() / la_rescale;
	la_y = (float) mpu6050->getAccelerationY() / la_rescale;
	la_z = (float) mpu6050->getAccelerationZ() / la_rescale;

	if(calibated == TRUE)
	     {
	       //Adding up samples to get cummulated data
	       if(count < SAMPLE_WINDOW_SIZE + 1)
	       {
		 cav_x = cav_x + av_x;
		 cav_y = cav_y + av_y;
		 cav_z = cav_z + av_z;

		 cla_x = cla_x + la_x;
		 cla_y = cla_y + la_y;
		 cla_z = cla_z + la_z;

		 count++;
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

		 //First inetgration to get angular angles
		 cap_x = pap_x + pav_x + (ssav_x - pav_x)/SAMPLE_INTERVAL_TIME;
		 cap_y = pap_y + pav_y + (ssav_y - pav_y)/SAMPLE_INTERVAL_TIME;
		 cap_z = pap_z + pav_z + (ssav_z - pav_z)/SAMPLE_INTERVAL_TIME;

		 //First integration to get linear velocity
		 clv_x = plv_x + pla_x + (ssla_x - pla_x)/SAMPLE_INTERVAL_TIME;
		 clv_y = plv_y + pla_y + (ssla_y - pla_y)/SAMPLE_INTERVAL_TIME;
		 clv_z = plv_z + pla_z + (ssla_z - pla_z)/SAMPLE_INTERVAL_TIME;

		 //second integration to get linear position
		 clp_x = plp_x + plv_x + (clv_x - plv_x)/SAMPLE_INTERVAL_TIME;
		 clp_y = plp_y + plv_y + (clv_y - plv_y)/SAMPLE_INTERVAL_TIME;
		 clp_z = plp_z + plv_z + (clv_z - plv_z)/SAMPLE_INTERVAL_TIME;

		 count = 0;
	       }
	     }
	     else
	     {
	       //Adding up samples to get cummulated data
	       if(count < CALIBRATE_SAMPLE_SIZE + 1)
	       {
		 cav_x = cav_x + av_x;
		 cav_y = cav_y + av_y;
		 cav_z = cav_z + av_z;

		 cla_x = cla_x + la_x;
		 cla_y = cla_y + la_y;
		 cla_z = cla_z + la_z;

		 count++;
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
		 ROS_INFO("Calibrated Angular: [%f, %f, %f]", bav_x, bav_y, bav_z);
		 ROS_INFO("Calibrated Linear: [%f, %f, %f]", bla_x, bla_y, bla_z);
	       }
	     }

	ROS_INFO("Angular:%i %i [%f, %f, %f]", calibated, count, cav_x, cav_y, cav_z);
	ROS_INFO("Linear:%i %i [%f, %f, %f]", calibated, count, cla_x, cla_y, cla_z);
}

int main(int argc, char** argv) {
	//double default_av_rescale = 131.0;
	//double default_la_rescale = 16384.0 / 9.807;

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

	signal(SIGINT, signal_handler);

	//Initialize devices
	I2cPort *i2c0 = new I2cPort(0x68, 2);
	mpu6050 = new MPU6050(i2c0);

	i2c0->openConnection();

	if (i2c0->isConnectionOpen()) {
	mpu6050->initialize();
	mpu6050->setI2CBypassEnabled(1);
	} else {
	ROS_FATAL("Could not connect to IMU - Accelorometer & Gyro Sensor");
	ROS_BREAK();
	}

	I2cPort *i2c1 = new I2cPort(0x1E, 2);
	HMC5883L *hmc5883L = new HMC5883L(i2c1);

	i2c1->openConnection();

	if (i2c1->isConnectionOpen()) {	hmc5883L->initialize();	}
	else {
		ROS_FATAL("Could not connect to IMU - Magnetometer Sensor");
		ROS_BREAK();
	}

	ros::init(argc, argv, "imu_raw");
	ros::NodeHandle node;
	ros::NodeHandle tnode;
	tf::TransformBroadcaster imu_broadcaster;

	node.getParam("rabbit_imu/av_rescale", av_rescale);
	node.getParam("rabbit_imu/la_rescale", la_rescale);

	ros::Publisher pub1 = node.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
	ros::Publisher pub2 = node.advertise<sensor_msgs::MagneticField>("imu/mag", 10);

	ros::Publisher pub3 = node.advertise<geometry_msgs::Vector3Stamped>("imu/avel", 100);
	ros::Publisher pub4 = node.advertise<geometry_msgs::Vector3Stamped>("imu/aangle", 100);

	ros::Publisher pub5 = node.advertise<geometry_msgs::Vector3Stamped>("imu/lacc", 100);
	ros::Publisher pub6 = node.advertise<geometry_msgs::Vector3Stamped>("imu/lvel", 100);
	ros::Publisher pub7 = node.advertise<geometry_msgs::Vector3Stamped>("imu/lpos", 100);

	ros::Timer timer1 = tnode.createTimer(ros::Duration(SAMPLE_INTERVAL_TIME), filter_imu_data);

	ros::Rate rate(20);  // hz

	ROS_INFO("IMU Node to send raw data started...");

while(ros::ok()) 
{
	    ros::Time current_time = ros::Time::now();

	    sensor_msgs::Imu msg1;
	    msg1.header.stamp = current_time;
	    msg1.header.frame_id = "imu";  // no frame

	    // Read gyroscope values.
	    // At default sensitivity of 250deg/s we need to scale by 131.
	    msg1.angular_velocity.x = (float) av_x;
	    msg1.angular_velocity.y = (float) av_y;
	    msg1.angular_velocity.z = (float) av_z;

	    // Read accelerometer values.
	    // At default sensitivity of 2g we need to scale by 16384.
	    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
	    // But! Imu msg docs say acceleration should be in m/2 so need to *9.807
	    msg1.linear_acceleration.x = (float) la_x;
	    msg1.linear_acceleration.y = (float) la_y;
	    msg1.linear_acceleration.z = (float) la_z;

	    sensor_msgs::MagneticField msg2;
	    msg2.header.stamp = current_time;
	    msg2.header.frame_id = '0';  // no frame

	    // Read magnetometer values.
	    msg2.magnetic_field.x = (float) hmc5883L->getMagnitudeX();
	    msg2.magnetic_field.y = (float) hmc5883L->getMagnitudeY();
	    msg2.magnetic_field.z = (float) hmc5883L->getMagnitudeZ();

	    imu_yaw = 0.0;

	    //since all IMU is 6DOF we'll need a quaternion created from yaw
	    geometry_msgs::Quaternion imu_quat = tf::createQuaternionMsgFromYaw(imu_yaw);

	    //first, we'll publish the transform over tf
	    geometry_msgs::TransformStamped imu_trans;
	    imu_trans.header.stamp = ros::Time::now();
	    //imu_trans.header.stamp = prev_measured_time;
	    imu_trans.header.frame_id = "base_link";
	    imu_trans.child_frame_id = "imu";

	    imu_trans.transform.translation.x = 0.5;
	    imu_trans.transform.translation.y = 0.5;
	    imu_trans.transform.translation.z = 0.0;
	    imu_trans.transform.rotation = imu_quat;

    	    geometry_msgs::Vector3Stamped msg3;
	    msg3.header.stamp = current_time;
	    msg3.header.frame_id = '0';  // no frame
	    msg3.vector.x = av_x;
	    msg3.vector.y = av_y;
	    msg3.vector.z = av_z;

    	    geometry_msgs::Vector3Stamped msg4;
	    msg4.header.stamp = current_time;
	    msg4.header.frame_id = '0';  // no frame
	    msg4.vector.x = cap_x;
	    msg4.vector.y = cap_y;
	    msg4.vector.z = cap_z;

    	    geometry_msgs::Vector3Stamped msg5;
	    msg5.header.stamp = current_time;
	    msg5.header.frame_id = '0';  // no frame
	    msg5.vector.x = la_x;
	    msg5.vector.y = la_y;
	    msg5.vector.z = la_z;

    	    geometry_msgs::Vector3Stamped msg6;
	    msg6.header.stamp = current_time;
	    msg6.header.frame_id = '0';  // no frame
	    msg6.vector.x = clv_x;
	    msg6.vector.y = clv_y;
	    msg6.vector.z = clv_z;

    	    geometry_msgs::Vector3Stamped msg7;
	    msg7.header.stamp = current_time;
	    msg7.header.frame_id = '0';  // no frame
	    msg7.vector.x = clp_x;
	    msg7.vector.y = clp_y;
	    msg7.vector.z = clp_z;

	    //send the transform
	    imu_broadcaster.sendTransform(imu_trans);

	    // Pub & sleep.
	    pub1.publish(msg1);
	    pub2.publish(msg2);

	    pub3.publish(msg3);
	    pub4.publish(msg4);

	    pub5.publish(msg5);
	    pub6.publish(msg6);
	    pub7.publish(msg7);

	    ros::spinOnce();
	    //ros::spin();
	    rate.sleep();
}

    delete i2c0;
    delete i2c1;
    delete mpu6050;
    delete hmc5883L;

    return 0;
}

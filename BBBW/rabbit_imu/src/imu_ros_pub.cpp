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

using namespace cacaosd_i2cport;
using namespace cacaosd_mpu6050;
using namespace cacaosd_hmc5883l;

int ctrl;

double imu_yaw;

int main(int argc, char** argv) {
    double default_av_rescale = 131.0;
    double default_la_rescale = 16384.0 / 9.807;

    double av_rescale;
    double la_rescale;

    signal(SIGINT, signal_handler);

    //Initialize devices
    I2cPort *i2c0 = new I2cPort(0x68, 1);
    MPU6050 *mpu6050 = new MPU6050(i2c0);

    i2c0->openConnection();

    if (i2c0->isConnectionOpen()) {
        mpu6050->initialize();
	mpu6050->setI2CBypassEnabled(1);
    } else {
	ROS_FATAL("Could not connect to IMU Sensor.");
	ROS_BREAK();
    }

    I2cPort *i2c1 = new I2cPort(0x1E, 1);
    HMC5883L *hmc5883L = new HMC5883L(i2c1);

    i2c1->openConnection();

    if (i2c1->isConnectionOpen()) {
        hmc5883L->initialize();
    }
    else {
	ROS_FATAL("Could not connect to IMU Sensor 2.");
	ROS_BREAK();
    }

     ros::init(argc, argv, "imu_raw");
     ros::NodeHandle node;
     tf::TransformBroadcaster imu_broadcaster;

     node.getParam("imu_node/rabbit_imu/av_rescale", av_rescale);
     node.getParam("imu_node/rabbit_imu/la_rescale", la_rescale);

     ros::Publisher pub1 = node.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
     ros::Publisher pub2 = node.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
     ros::Rate rate(20);  // hz

     ROS_INFO("IMU Node to send raw data started...");

while(ros::ok()) {
	    sensor_msgs::Imu msg1;
	    msg1.header.stamp = ros::Time::now();
	    msg1.header.frame_id = "imu";  // no frame

	    msg1.orientation.x=0;
	    msg1.orientation.y=0;
	    msg1.orientation.z=0;
	    msg1.orientation.w=1;

	    // Read gyroscope values.
	    // At default sensitivity of 250deg/s we need to scale by 131.
	    msg1.angular_velocity.x = (float) mpu6050->getAccelerationZ() / av_rescale;
	    msg1.angular_velocity.y = (float) mpu6050->getAngularVelocityY() / av_rescale;
	    msg1.angular_velocity.z = (float) mpu6050->getAngularVelocityZ() / av_rescale;

	    // Read accelerometer values.
	    // At default sensitivity of 2g we need to scale by 16384.
	    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
	    // But! Imu msg docs say acceleration should be in m/2 so need to *9.807
	    msg1.linear_acceleration.x = (float) mpu6050->getAccelerationX() / la_rescale;
	    msg1.linear_acceleration.y = (float) mpu6050->getAccelerationY() / la_rescale;
	    msg1.linear_acceleration.z = (float) mpu6050->getAccelerationZ() / la_rescale;

	    sensor_msgs::MagneticField msg2;
	    msg2.header.stamp = ros::Time::now();
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
	    imu_trans.header.frame_id = "base_link";
	    imu_trans.child_frame_id = "imu";

	    imu_trans.transform.translation.x = 0.5;
	    imu_trans.transform.translation.y = 0.5;
	    imu_trans.transform.translation.z = 0.0;
	    imu_trans.transform.rotation = imu_quat;

	    //send the transform
	    imu_broadcaster.sendTransform(imu_trans);

	    // Pub & sleep.
	    pub1.publish(msg1);
	    pub2.publish(msg2);
	    ros::spinOnce();
	    rate.sleep();
}

    delete i2c0;
    delete i2c1;
    delete mpu6050;
    delete hmc5883L;

    return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <rabbit_imu/MPU9250.h>

using namespace cacaosd_i2cport;
using namespace mpu9250;

float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
int ctrl;

int main(int argc, char** argv) {
    double default_av_rescale = 131.0;
    double default_la_rescale = 16384.0 / 9.807;

    double av_rescale;
    double la_rescale;

//    signal(SIGINT, signal_handler);

    //Initialize devices
    I2cPort *i2c0 = new I2cPort(0x68, 2);
    MPU9250 *_mpu9250 = new MPU9250(i2c0);

    i2c0->openConnection();

    if (i2c0->isConnectionOpen()) {
	  // start communication with IMU and 
	  // set the accelerometer and gyro ranges.
	  // ACCELEROMETER 2G 4G 8G 16G
	  // GYRO 250DPS 500DPS 1000DPS 2000DPS
	  ctrl = _mpu9250->begin(ACCEL_RANGE_8G,GYRO_RANGE_1000DPS);
    } else {
	ROS_FATAL("Could not connect to IMU Sensor.");
	ROS_BREAK();
    }

     ros::init(argc, argv, "imu_raw");
     ros::NodeHandle node;

     node.getParam("rabbit_imu/av_rescale", av_rescale);
     node.getParam("rabbit_imu/la_rescale", la_rescale);

     ros::Publisher pub1 = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
     ros::Publisher pub2 = node.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
     ros::Rate rate(100);  // hz

     ROS_INFO("IMU Node to send raw data started...");

while(ros::ok()) {
	    sensor_msgs::Imu msg1;
	    msg1.header.stamp = ros::Time::now();
	    msg1.header.frame_id = '0';  // no frame

	    // Read gyroscope values.
	    // At default sensitivity of 1000deg/s we need to scale by 131.
	    // get the gyro data (rad/s)
	    _mpu9250->getGyro(&gx, &gy, &gz); 
//	    msg1.angular_velocity.x = (float) gx / av_rescale;
//	    msg1.angular_velocity.y = (float) gy / av_rescale;
//	    msg1.angular_velocity.z = (float) gz / av_rescale;
	    msg1.angular_velocity.x = (float) gx;
	    msg1.angular_velocity.y = (float) gy;
	    msg1.angular_velocity.z = (float) gz;

	    // Read accelerometer values.
	    // At default sensitivity of 8g we need to scale by 16384.
	    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
	    // But! Imu msg docs say acceleration should be in m/2 so need to *9.807   
	    _mpu9250->getAccel(&ax, &ay, &az);    	 
//	    msg1.linear_acceleration.x = (float) ax / la_rescale;
//	    msg1.linear_acceleration.y = (float) ay / la_rescale;
//	    msg1.linear_acceleration.z = (float) az / la_rescale;
	    msg1.linear_acceleration.x = (float) ax;
	    msg1.linear_acceleration.y = (float) ay;
	    msg1.linear_acceleration.z = (float) az;

	    sensor_msgs::MagneticField msg2;
	    msg2.header.stamp = ros::Time::now();
	    msg2.header.frame_id = '0';  // no frame

	    // Read magnetometer values.
	    // get the magnetometer data (uT)
	    _mpu9250->getMag(&hx, &hy, &hz);
	    msg2.magnetic_field.x = (float) hx;
	    msg2.magnetic_field.y = (float) hy;
	    msg2.magnetic_field.z = (float) hz;

	    // Pub & sleep.
	    pub1.publish(msg1);
	    pub2.publish(msg2);
	    ros::spinOnce();
	    rate.sleep();
}

    delete i2c0;
//    delete i2c1;
    delete _mpu9250;
//    delete hmc5883L;

    return 0;
}


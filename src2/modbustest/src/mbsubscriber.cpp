#include <ros/ros.h>
#include <modbustest/readinputregisters.h>
#include <sstream>

int m1encoder_slaveid;
int m2encoder_slaveid;
int m3encoder_slaveid;
int m4encoder_slaveid;

int m1encoder_index;
int m2encoder_index;
int m3encoder_index;
int m4encoder_index;


void modbusreadCallback(const modbustest::readinputregisters::ConstPtr& msg)
{
unsigned int slaveid = msg->slaveid;
unsigned int read_input_registers_count = msg->read_input_registers_count;
unsigned int read_input_registers_start_address = msg->read_input_registers_start_address;

int encoder1_counts = 0;
int encoder2_counts = 0;

	if(slaveid == m1encoder_slaveid)
	{
		encoder1_counts=msg->read_input_registers[m1encoder_index];
		ROS_INFO("slave %i index:%i Data::%i",slaveid, m1encoder_index, encoder1_counts);
	}
	else if(slaveid == m2encoder_slaveid)
	{
		encoder2_counts=msg->read_input_registers[m2encoder_index];
		ROS_INFO("slave %i index:%i Data::%i",slaveid, m2encoder_index, encoder2_counts);
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_encoder");

  ros::NodeHandle n;

	//Getting slave details, coil counts and starting address register details from ros parameters
	n.param<int>("mencoder/m1encoder_slaveid", m1encoder_slaveid, 51);
	n.param<int>("mencoder/m2encoder_slaveid", m2encoder_slaveid, 53);
	n.param<int>("mencoder/m3encoder_slaveid", m3encoder_slaveid, 0);
	n.param<int>("mencoder/m4encoder_slaveid", m4encoder_slaveid, 0);

	n.param<int>("mencoder/m1encoder_index", m1encoder_index, 7);
	n.param<int>("mencoder/m2encoder_index", m2encoder_index, 7);
	n.param<int>("mencoder/m3encoder_index", m3encoder_index, 0);
	n.param<int>("mencoder/m4encoder_index", m4encoder_index, 0);

	if((m1encoder_slaveid == 0) && (m2encoder_slaveid == 0) && (m3encoder_slaveid == 0) && (m4encoder_slaveid == 0))
	{
	ROS_ERROR("Error: Parameters for encoder - slave id  not loaded sucessfully!");
	return 0;
	}
  ros::Subscriber chatter_sub = n.subscribe("modbus_read_input_registers", 1,modbusreadCallback);

  ros::spin();

  return 0;
}

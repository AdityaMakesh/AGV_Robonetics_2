#include <ros/ros.h>
#include <modbustest/readinputregisters.h>
#include <sstream>
#include <iostream>

int m_slaveid;
int m_index;


void modbusreadCallback(const modbustest::readinputregisters::ConstPtr& msg)
{
unsigned int slaveid = msg->slaveid;
unsigned int read_input_registers_count = msg->read_input_registers_count;
unsigned int read_input_registers_start_address = msg->read_input_registers_start_address;

int adc_value = 0;
float avoltage = 0.0;
std::stringstream ss;

	if(slaveid == m_slaveid)
	{
		adc_value=msg->read_input_registers[m_index];
		avoltage = ((adc_value*5*3.33)/1024);
		ROS_INFO("slave %i index:%i Data::%i : %f V",slaveid, m_index, adc_value, (float) avoltage);
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_voltage");

  ros::NodeHandle n;

	//Getting slave details, coil counts and starting address register details from ros parameters

	n.param<int>("mvoltage/slaveid", m_slaveid, 0);
	n.param<int>("mvoltage/index", m_index, 7);


	if(m_slaveid == 0)
	{
	ROS_ERROR("Error: Parameters for voltage board - slave id  not loaded sucessfully!");
	return 0;
	}
  ros::Subscriber chatter_sub = n.subscribe("modbus_read_input_registers", 1,modbusreadCallback);

  ros::spin();

  return 0;
}

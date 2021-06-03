#include <ros/ros.h>
#include <modbustest/readinputregisters.h>
#include <sstream>

#define SHARP_SLAVE_ID1 51

int sharp_slaveid;
int sharp_index;

void modbusreadCallback(const modbustest::readinputregisters::ConstPtr& msg)
{
unsigned int slaveid = msg->slaveid;
unsigned int read_input_registers_count = msg->read_input_registers_count;
unsigned int read_input_registers_start_address = msg->read_input_registers_start_address;

int sharp_distance = 0;

	if(slaveid == SHARP_SLAVE_ID1)
	{
	sharp_distance=msg->read_input_registers[sharp_index];
	ROS_INFO("slave %i index:%i Data::%i",slaveid, sharp_index, sharp_distance);
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_sharp");

  ros::NodeHandle n;

	//Getting slave details, coil counts and starting address register details from ros parameters
	n.param<int>("msharp/sharp_slaveid", sharp_slaveid, 51);
	n.param<int>("msharp/sharp_index", sharp_index, 1);

	if(sharp_slaveid == 0)
	{
	ROS_ERROR("Error: Parameters for sharp ir - slave id  not loaded sucessfully!");
	return 0;
	}
  ros::Subscriber chatter_sub = n.subscribe("modbus_read_input_registers", 1,modbusreadCallback);

  ros::spin();

  return 0;
}

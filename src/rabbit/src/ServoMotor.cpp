#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>
#include <sstream>
#include <cstdlib>
#include <rabbit/ServoMotor.h>

ServoMotor::RhinoInterface::RhinoInterface(const char * new_serial_port_)
{
	port_name_ = new_serial_port_;
	serial_port_ = new sereal::SerealPort();
}

ServoMotor::RhinoInterface::~RhinoInterface()
{
	delete serial_port_;
}

//! Open the serial port
int ServoMotor::RhinoInterface::openSerialPort()
{
	try{ serial_port_->open(port_name_.c_str(), 9600);}
	catch(sereal::Exception& e){ return(-1); }

return(0);
}

//! Close the serial port
int ServoMotor::RhinoInterface::closeSerialPort()
{
	try{ serial_port_->close();}
	catch(sereal::Exception& e){ return(-1); }

return(0);
}

int ServoMotor::RhinoInterface::readMotorSpeed()
{
char data_buffer[15];
int speed=-1;
std::stringstream ss;

	ss<<"S\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

	try{ serial_port_->readLine(data_buffer, 15,5); }
	catch(sereal::Exception& e){ return(-1); }

	str = std::string(data_buffer);
        std::size_t found=-1;
        found = str.find(":");
        if(found == 0)
        {
        str.replace(found,found+2," ");
        speed = atoll(str.c_str());
	}

return(speed);
}

int ServoMotor::RhinoInterface::writeMotorSpeed(int speed)
{
std::stringstream ss;

	ss<<"S"<<speed<<"\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

unsigned int ServoMotor::RhinoInterface::getMaxMotorSpeed()
{
char data_buffer[15];
unsigned int speed=-1;
std::stringstream ss;

	ss<<"M\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

	try{ serial_port_->readLine(data_buffer,15,5); }
	catch(sereal::Exception& e){ return(-1); }

	str = std::string(data_buffer);
        std::size_t found=-1;
        found = str.find(":");
        if(found == 0)
        {
        str.replace(found,found+2," ");
        speed = atoll(str.c_str());
	}

return(speed);
}

unsigned int ServoMotor::RhinoInterface::setMaxMotorSpeed(unsigned int speed)
{
std::stringstream ss;

	ss<<"M"<<speed<<"\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

unsigned int ServoMotor::RhinoInterface::readDamping()
{
char data_buffer[15];
unsigned int damp=-1;
std::stringstream ss;

	ss<<"D\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

	try{ serial_port_->readLine(data_buffer, 15,5); }
	catch(sereal::Exception& e){ return(-1); }

	str = std::string(data_buffer);
        std::size_t found=-1;
        found = str.find(":");
        if(found == 0)
        {
        str.replace(found,found+2," ");
        damp = atoi(str.c_str());
	}

return(damp);
}

int ServoMotor::RhinoInterface::writeDamping(unsigned int value)
{
std::stringstream ss;

	ss<<"D"<<value<<"\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

int ServoMotor::RhinoInterface::loadFactorySettings()
{
	std::stringstream ss;
	ss<<"Y\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

long long int ServoMotor::RhinoInterface::getPositionEncoder()
{
char data_buffer[15];
long long int encoder=-1;

	std::stringstream ss;
	ss<<"P\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

	try{ serial_port_->readLine(data_buffer, 15,5); }
	catch(sereal::Exception& e){ return(-1); }

	str = std::string(data_buffer);
        std::size_t found=-1;
        found = str.find(":");
        if(found == 0)
        {
        str.replace(found,found+2," ");
        encoder = atoll(str.c_str());
	}

return(encoder);
}

int ServoMotor::RhinoInterface::setPositionEncoder(long long int encoder)
{
	std::stringstream ss;
	ss<<"P"<<encoder<<"\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

long long int ServoMotor::RhinoInterface::getAbsolutePostion()
{
char data_buffer[15];
long long int position=-1;

	std::stringstream ss;
	ss<<"G\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

	try{ serial_port_->readLine(data_buffer, 15,5);	}
	catch(sereal::Exception& e){ return(-1); }

	str = std::string(data_buffer);
        std::size_t found=-1;
        found = str.find(":");
        if(found == 0)
        {
        str.replace(found,found+2," ");
        position = atoll(str.c_str());
	}

return(position);
}

int ServoMotor::RhinoInterface::setAbsolutePostion(long long int position)
{
	std::stringstream ss;
	ss<<"G"<<position<<"\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

int ServoMotor::RhinoInterface::setRelativePostion(long long int position)
{
	std::stringstream ss;
	ss<<"R"<<position<<"\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

unsigned int ServoMotor::RhinoInterface::getFeedbackGain()
{
char data_buffer[15];
int gain=-1;
std::stringstream ss;
ss<<"A\r\n";
std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

	try{ serial_port_->readLine(data_buffer, 15,5); }
	catch(sereal::Exception& e){ return(-1); }

	str = std::string(data_buffer);
        std::size_t found=-1;
        found = str.find(":");
        if(found == 0)
        {
        str.replace(found,found+2," ");
        gain = atoi(str.c_str());
	}

return(gain);
}

int ServoMotor::RhinoInterface::setFeedbackGain(unsigned int gain)
{
	std::stringstream ss;
	ss<<"A"<<gain<<"\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }
return 0;
}

unsigned int ServoMotor::RhinoInterface::getProportionateGain()
{
char data_buffer[15];
unsigned int gain=-1;

	std::stringstream ss;
	ss<<"B\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

	try{ serial_port_->readLine(data_buffer, 15,5); }
	catch(sereal::Exception& e){ return(-1); }

	str = std::string(data_buffer);
        std::size_t found=-1;
        found = str.find(":");
        if(found == 0)
        {
        str.replace(found,found+2," ");
        gain = atoi(str.c_str());
	}

return(gain);
}

int ServoMotor::RhinoInterface::setProportionateGain(unsigned int gain)
{
std::stringstream ss;

	ss<<"B"<<gain<<"\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

unsigned int ServoMotor::RhinoInterface::getIntegralGain()
{
char data_buffer[15];
long long int gain=-1;
std::stringstream ss;

	ss<<"C\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

	try{ serial_port_->readLine(data_buffer, 15,5); }
	catch(sereal::Exception& e){ return(-1); }
	str = std::string(data_buffer);

        std::size_t found=-1;
        found = str.find(":");
        if(found == 0)
        {
        str.replace(found,found+2," ");
        gain = atoi(str.c_str());
	}

return(gain);
}

int ServoMotor::RhinoInterface::setIntegralGain(unsigned int gain)
{
std::stringstream ss;

	ss<<"C"<<gain<<"\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

int ServoMotor::RhinoInterface::autoCalibrate()
{
std::stringstream ss;

	ss<<"X\r\n";
	std::string str = ss.str();

	try{ serial_port_->write(str.c_str(), (int)str.length()); }
	catch(sereal::Exception& e){ return(-1); }

return 0;
}

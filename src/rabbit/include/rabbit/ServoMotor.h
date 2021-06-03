#include <sereal/sereal.h>

#define NODE_VERSION 			1.0
# define M_PI           3.14159265358979323846  /* pi */

namespace ServoMotor
{

	class RhinoInterface
	{
		public:
		RhinoInterface(const char * serial_port_);
		~RhinoInterface();
		//! Open the serial port
		int openSerialPort();
		//! Close the serial port
		int closeSerialPort();

		//Read write motor speed and direction
		//Value Ranges from -255 to +255
		int readMotorSpeed();
		int writeMotorSpeed(int speed);

		//Read write motor maximum speed
		//Value Ranges from 0 to +255
		unsigned int getMaxMotorSpeed();
		unsigned int setMaxMotorSpeed(unsigned int speed);

		//Read write Speed Damping
		//Value Ranges from 0 to 255 
		unsigned int readDamping();
		int writeDamping(unsigned int value);

		//Reset Factory Settings
		//No return value
		int loadFactorySettings();

		//Read write Position Encoder Values
		//Value Ranges from -2147483648 to +2147483647 
		long long int getPositionEncoder();
		int setPositionEncoder(long long int encoder);

		//Read write Absolute Position Values
		//Value Ranges from -2147483648 to +2147483647 
		long long int getAbsolutePostion();
		int setAbsolutePostion(long long int position);

		//Write Relative Position Values
		//Value Ranges from -2147483648 to +2147483647 
		int setRelativePostion(long long int position);

		//Read write speed feedback gain term
		//Values Ranges 0 to 32767
		unsigned int getFeedbackGain();
		int setFeedbackGain(unsigned int gain);

		//Read write P gain term
		//Values Ranges 0 to 32767
		unsigned int getProportionateGain();
		int setProportionateGain(unsigned int gain);

		//Read write I gain term
		//Values Ranges 0 to 32767
		unsigned int getIntegralGain();
		int setIntegralGain(unsigned int gain);

		//Motor Auto calibrate
		int autoCalibrate();

		private:
		//! Serial port to which the robot is connected
		std::string port_name_;
		//! sereal port object
		sereal::SerealPort * serial_port_;
		int packets_size_;
	};
}

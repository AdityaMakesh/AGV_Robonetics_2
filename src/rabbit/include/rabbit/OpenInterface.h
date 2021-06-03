#include <sereal/sereal.h>

#define NODE_VERSION 			1.0
# define M_PI           3.14159265358979323846  /* pi */

// Packets sizes
#define OI_PACKET_GROUP_1_SIZE 		1 	//! OI packets 7-16

// Positions
#define LEFT				0
#define RIGHT				1

// Delay after mode change in ms
#define OI_DELAY_MODECHANGE_MS		20

#define OI_OPCODE_STREAM = 145,
#define OI_OPCODE_PAUSE_STREAM = 150,

//  EYANTRA Dimensions
#define EYANTRA_BUMPER_X_OFFSET		0.001
#define EYANTRA_DIAMETER		0.051
#define EYANTRA_AXLE_LENGTH		0.150

#define  EYANTRA_MAX_LIN_VEL_MM_S	500
#define  EYANTRA_MAX_ANG_VEL_RAD_S	2  
#define  EYANTRA_MAX_RADIUS_MM		2000

//!  EYANTRA max encoder counts
#define  EYANTRA_MAX_ENCODER_COUNTS	30
//!  EYANTRA encoder pulses to meter constant
#define  EYANTRA_PULSES_TO_M		0.00544			//(2 * PI * Radius of wheel)/Encoder counts per revolution

#define MAX_PATH 			32

#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

namespace psgrobot
{
	//! OI op codes
	/*!
	 * Op codes for commands as specified by the psg robot Open Interface.
	 */
	typedef enum _OI_Opcode {
		// Command opcodes
		OI_OPCODE_START = 131,
		OI_OPCODE_BAUD = 132,
		OI_OPCODE_CONTROL = 133,
		OI_OPCODE_SAFE = 134,
		OI_OPCODE_FULL = 135,
		OI_OPCODE_POWER = 136
	} OI_Opcode;


	//! OI packet id
	/*!
	 * Packet ids for sensors as specified by the psg robot Open Interface.
	 */
	typedef enum _OI_Packet_ID {	
		// Sensor Packets 
		OI_PACKET_POSITION_ENCODER_1 = 24,
		OI_PACKET_POSITION_ENCODER_2 = 25,
		OI_PACKET_POSITION_ENCODER_3 = 26,
		OI_PACKET_POSITION_ENCODER_4 = 27
	} OI_Packet_ID;


	/*! \class OpenInterface OpenInterface.h "inc/OpenInterface.h"
	 *  \brief C++ class implementation of the psgrobot OI.
	*/
	class OpenInterface
	{
		public:
	
		//! Constructor
		/*!
		 * By default the constructor will set the  EYANTRA to read only the encoder counts (for odometry).
		 *
		 *  \param new_serial_port    Name of the serial port to open.
		 *
		 *  \sa setSensorPackets()
		 */
		//OpenInterface(const char * new_serial_port);
		OpenInterface(const char * serial_porta, const char * serial_portb);
		//! Destructor
		~OpenInterface();
	
		//! Open the serial port
		/*!
		 *  \param full_control    Whether to set the  EYANTRA on OImode full or not.
		 */
		int openSerialPort(bool full_control);
		//! Close the serial port
		int closeSerialPort();
	
		//! Power down the  EYANTRA.
		int powerDown();
	
		//! Set sensor packets
		/*!
		*  Set the sensor packets one wishes to read from the  EYANTRA. By default the constructor will set the  EYANTRA to read only the encoder counts (for odometry). 
		*
		*  \param new_sensor_packets  	Array of sensor packets to read.
		*  \param new_num_of_packets  	Number of sensor packets in the array.
		*  \param new_buffer_size		Size of the resulting sensor data reply.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int setSensorPackets(OI_Packet_ID * new_sensor_packets, int new_num_of_packets, size_t new_buffer_size);
		int setcharSensorPackets(char * new_sensor_packets, int new_num_of_packets, int new_buffer_size);
		//! Read sensor packets
		/*!
		*  Requested the defined sensor packets from the  EYANTRA. If you need odometry and you requested encoder data you need to call calculateOdometry() afterwords.
		*
		*  \param timeout		Timeout in milliseconds.
		*
		* \sa calculateOdometry()
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int getSensorPackets(int timeout);
		
		//! Stream sensor packets. NOT TESTED
		int streamSensorPackets();
		//! Start stream. NOT TESTED
		int startStream();
		//! Stom stream. NOT TESTED
		int stopStream();
	
		//! Calculate  EYANTRA odometry. Call after reading encoder pulses.
		void calculateOdometry();
	
		//! Drive
		/*!
		*  Send velocity commands to  EYANTRA.
		*
		*  \param linear_speed  	Linear speed.
		*  \param angular_speed  	Angular speed.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int drive(double linear_speed, double angular_speed);
		//! Drive direct
		/*!
		*  Send velocity commands to  EYANTRA.
		*
		*  \param left_speed  	Left wheel speed.
		*  \param right_speed  	Right wheel speed.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int driveDirect(int left_speed, int right_speed);
		//! Drive PWM
		/*!
		*  Set the motors pwms. NOT IMPLEMENTED
		*
		*  \param left_pwm  	Left wheel motor pwm.
		*  \param right_pwm  	Right wheel motor pwm.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int drivePWM(int left_pwm, int right_pwm);

		//! Set brushes motors pwms
		/*!
		*  Set the brushes motors pwms. This is very interesting. One could disconnect the motors and plug other actuators that could be controller over pwm on the  EYANTRA.
		*
		*  \param main_brush 	Main brush motor pwm.
		*  \param side_brush  	Side brush motor pwm.
		*  \param vacuum  		Vacuum motor pwm.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int brushesPWM(char main_brush, char side_brush, char vacuum);
	
		//! Set the  EYANTRA in cleaning mode. Returns the OImode to safe.
		int clean();
		//! Set the  EYANTRA in max cleaning mode. Returns the OImode to safe.
		int max();
		//! Set the  EYANTRA in spot cleaning mode. Returns the OImode to safe.
		int spot();
		//! Sends the  EYANTRA to the dock. Returns the OImode to safe.
		int goDock();
	
		//! Set digit leds
		/*!
		*  Set the digit leds on the  EYANTRA, the ones on the clock. Digits are ordered from left to right on the robot, 3, 2, 1, 0.
		*
		*  \param digit3 		Digit 3
		*  \param digit2		Digit 2
		*  \param digit1		Digit 1
		*  \param digit0		Digit 0
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int setDigitLeds(unsigned char data);
	
		//! Current operation mode, one of  EYANTRA_MODE_'s
		unsigned char OImode_;
	
		//! Sends the  EYANTRA to the dock. Returns the OImode to safe.
		void resetOdometry();
		void setOdometry(double new_x, double new_y, double new_yaw);
	
		//!  EYANTRA odometry x
		double odometry_x_;
		//!  EYANTRA odometry y
		double odometry_y_;
		//!  EYANTRA odometry yaw
		double odometry_yaw_;
	
		bool wall_;			//! Wall detected.
		bool virtual_wall_;		//! Virtual wall detected.
		bool cliff_[4];			//! Cliff sensors. Indexes: LEFT FRONT_LEFT FRONT_RIGHT RIGHT
		bool bumper_[2];		//! Bumper sensors. Indexes: LEFT RIGHT
		bool ir_bumper_[6];		//! IR bumper sensors. Indexes: LEFT FRONT_LEFT CENTER_LEFT CENTER_RIGHT FRONT_RIGHT RIGHT
		bool wheel_drop_[2];		//! Wheel drop sensors: Indexes: LEFT RIGHT
		int wall_signal_;		//! Wall signal.
		int cliff_signal_[4];		//! CLiff sensors signal. Indexes: LEFT FRONT_LEFT FRONT_RIGHT RIGHT
		int ir_bumper_signal_[6];	//! IR bumper sensors signal. Indexes: LEFT FRONT_LEFT CENTER_LEFT CENTER_RIGHT FRONT_RIGHT RIGHT
		unsigned char ir_char_[3];	//! IR characters received. Indexes: OMNI LEFT RIGHT
	
		bool buttons_[8];		//! Buttons. Indexes: BUTTON_CLOCK BUTTON_SCHEDULE BUTTON_DAY BUTTON_HOUR BUTTON_MINUTE BUTTON_DOCK BUTTON_SPOT BUTTON_CLEAN
	
		unsigned char dirt_detect_;	//! Dirt detected
	
		int motor_current_[4];		//! Motor current. Indexes: LEFT RIGHT MAIN_BRUSH SIDE_BRUSH
		bool overcurrent_[4];		//! Motor overcurrent. Indexes: LEFT RIGHT MAIN_BRUSH SIDE_BRUSH
	
		unsigned char charging_state_;	//! One of OI_CHARGING_'s
		bool power_cord_;		//! Whether the  EYANTRA is connected to the power cord or not.
		bool dock_;			//! Whether the  EYANTRA is docked or not.
		float voltage_;			//! Battery voltage in volts.
		float current_;			//! Battery current in amps.
		char temperature_;		//! Battery temperature in C degrees.
		float charge_;			//! Battery charge in Ah.
		float capacity_;		//! Battery capacity in Ah
	
		int stasis_;			//! 1 when the robot is going forward, 0 otherwise

		private:
	
		//! Parse data
		/*!
		*  Data parsing function. Parses data comming from the  EYANTRA.
		*
		*  \param buffer  			Data to be parsed.
		*  \param buffer_length  	Size of the data buffer.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int parseSensorPackets(unsigned char * buffer, size_t buffer_length);	
		int parseBumpersAndWheeldrops(unsigned char * buffer, int index);
		int parseWall(unsigned char * buffer, int index);
		int parseLeftCliff(unsigned char * buffer, int index);
		int parseFrontLeftCliff(unsigned char * buffer, int index);
		int parseFrontRightCliff(unsigned char * buffer, int index);
		int parseRightCliff(unsigned char * buffer, int index);	
		int parseVirtualWall(unsigned char * buffer, int index);
		int parseOvercurrents(unsigned char * buffer, int index);
		int parseDirtDetector(unsigned char * buffer, int index);
		int parseIrOmniChar(unsigned char * buffer, int index);
		int parseButtons(unsigned char * buffer, int index);
		int parseDistance(unsigned char * buffer, int index);
		int parseAngle(unsigned char * buffer, int index);
		int parseChargingState(unsigned char * buffer, int index);
		int parseVoltage(unsigned char * buffer, int index);
		int parseCurrent(unsigned char * buffer, int index);
		int parseTemperature(unsigned char * buffer, int index);
		int parseBatteryCharge(unsigned char * buffer, int index);
		int parseBatteryCapacity(unsigned char * buffer, int index);
		int parseWallSignal(unsigned char * buffer, int index);
		int parseLeftCliffSignal(unsigned char * buffer, int index);
		int parseFrontLeftCliffSignal(unsigned char * buffer, int index);
		int parseFontRightCliffSignal(unsigned char * buffer, int index);
		int parseRightCliffSignal(unsigned char * buffer, int index);
		int parseChargingSource(unsigned char * buffer, int index);
		int parseOiMode(unsigned char * buffer, int index);
		int parseSongNumber(unsigned char * buffer, int index);
		int parseSongPlaying(unsigned char * buffer, int index);
		int parseNumberOfStreamPackets(unsigned char * buffer, int index);
		int parseRequestedVelocity(unsigned char * buffer, int index);
		int parseRequestedRadius(unsigned char * buffer, int index);
		int parseRequestedRightVelocity(unsigned char * buffer, int index);
		int parseRequestedLeftVelocity(unsigned char * buffer, int index);
		int parseRightEncoderCounts(unsigned char * buffer, int index);
		int parseLeftEncoderCounts(unsigned char * buffer, int index);
		int parseLightBumper(unsigned char * buffer, int index);
		int parseLightBumperLeftSignal(unsigned char * buffer, int index);
		int parseLightBumperFrontLeftSignal(unsigned char * buffer, int index);
		int parseLightBumperCenterLeftSignal(unsigned char * buffer, int index);
		int parseLightBumperCenterRightSignal(unsigned char * buffer, int index);
		int parseLightBumperFrontRightSignal(unsigned char * buffer, int index);
		int parseLightBumperRightSignal(unsigned char * buffer, int index);
		int parseIrCharLeft(unsigned char * buffer, int index);
		int parseIrCharRight(unsigned char * buffer, int index);
		int parseLeftMotorCurrent(unsigned char * buffer, int index);
		int parseRightMotorCurrent(unsigned char * buffer, int index);
		int parseMainBrushMotorCurrent(unsigned char * buffer, int index);
		int parseSideBrushMotorCurrent(unsigned char * buffer, int index);
		int parseStasis(unsigned char * buffer, int index);
	
		//! Buffer to signed int
		/*!
		*  Parsing helper function. Converts 2 bytes of data into a signed int value. 
		*
		*  \param buffer  	Data buffer.
		*  \param index  	Position in the buffer where the value is.
		*
		*  \sa buffer2unsigned_int()
		*
		*  \return A signed int value.
		*/
		int buffer2signed_int(unsigned char * buffer, int index);
		//! Buffer to unsigned int
		/*!
		*  Parsing helper function. Converts 2 bytes of data into an unsigned int value. 
		*
		*  \param buffer  	Data buffer.
		*  \param index  	Position in the buffer where the value is.
		*
		*  \sa buffer2signed_int()
		*
		*  \return An unsigned int value.
		*/
		int buffer2unsigned_int(unsigned char * buffer, int index);
	
		//! Start OI
		/*!
		*  Start the OI, change to  EYANTRA to a OImode that allows control.
		*
		*  \param full_control    Whether to set the  EYANTRA on OImode full or not.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int startOI(bool full_control);
		//! Send OP code
		/*!
		*  Send an OP code to  EYANTRA.
		*
		*  \param code  			OP code to send.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		int sendOpcode(OI_Opcode code);
	
		//! Serial port to which the robot is connected
		std::string port_name_a;
		std::string port_name_b;
		//! sereal port object
		sereal::SerealPort * serial_port_a;
		sereal::SerealPort * serial_port_b;

		//! Stream variable. NOT TESTED
		bool stream_defined_;
		
		//! Number of packets
		int num_of_packets_;
		//! Array of packets
		OI_Packet_ID * isensor_packets_;
		char * sensor_packets_;
		//! Total size of packets
		int packets_size_;
	
		//! Amount of distance travelled since last reading. Not being used, poor resolution. 
		int distance_;
		//! Amount of angle turned since last reading. Not being used, poor resolution. 
		int angle_;
		//! Delta encoder counts.
		int encoder_counts_[2];
		//! Last encoder counts reading. For odometry calculation.
		uint16_t last_encoder_counts_[2];
	};
}

// EOF

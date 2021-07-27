#include <AugmentedEncoder.h>
#include <SimPID.h>
#include <XM23.h>
#include <XM23Lib.h>

#include "frc/WPILib.h"
#include "rev/SparkMax.h"
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"
#include "ctre/Phoenix.h"

#include "adi/ADIS16470_IMU.h"

#include "cameraserver/CameraServer.h"
#include "cscore.h"

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include "tables/ITable.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "wpi/StringRef.h"
#include "wpi/ArrayRef.h"
#include <thread>
#include <string>
#include <chrono>



//#DEFINES

enum Constants {
	/**
	 * Which PID slot to pull gains from.  Starting 2018, you can choose
	 * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
	 */
	kSlotIdx = 0,

	/* Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
	 * For now we just want the primary one.
	 */
	kPIDLoopIdx = 0,

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	kTimeoutMs = 30
};




// General Definitions - DO NOT MODIFY

#define NEUTRAL 		0.0		//Stop Speed Command Value for Speed Controllers
#define MAXPOS  		3		//Number of Autonomous Starting Positions (default = 3, 2015 = 2)
#define MAXPROG 		8 		//Number of Autonomous Programs Per Starting Position

#define MAXDELAY      	5.0     //Maximum Autonomous Start Time Delay

#define BLINKONTIME 	0.25	//Blink Timer Control On Time		
#define BLINKOFFTIME 	0.50	//Blink	Timer Control Off Time (1 Second Cycle Time)

#define LIGHTON			frc::Relay::kOn
#define LIGHTOFF		frc::Relay::kOff

//Pneumatic Function Defines - DO NOT MODIFY
//A or true = Active Function   B or false = Default (Match Start) Function

//Single Valve Blocks

//48 - Shifter
#define HIGHGEAR			true		
#define LOWGEAR				false						//DEFAULT
	
//High Pressure Manifold

//Shooter Hood
#define SHOOTCLOSE			true						
#define SHOOTFAR			false						//DEFAULT

//Shooter Cage Tllt
#define TILTFORWARD			DoubleSolenoid48::OnA
#define TILTBACK			DoubleSolenoid48::OnB		//DEFAULT

//Intake Deploy
#define INTAKEOUT			DoubleSolenoid48::OnA		
#define INTAKEIN			DoubleSolenoid48::OnB 		//DEFAULT

//Climber Lock
#define LOCKON				DoubleSolenoid48::OnA 
#define LOCKOFF				DoubleSolenoid48::OnB		//DEFAULT

//Low Pressure Manifold

//

// Define constants for automatic smart selections - CAN BE MODIFIED IF NEEDED

#define LY_PLUS 		0.7
#define LY_MINUS 	   -0.7
#define LX_PLUS 		0.7
#define LX_MINUS 	   -0.7

#define RY_PLUS 		0.8
#define RY_MINUS 	   -0.8
#define RX_PLUS 		0.8
#define RX_MINUS 	   -0.8

#define PICKLEFT		1
#define PICKRIGHT		0

//DO MATH HERE!


//*********************** PID GAINS ***************************************************


//#define ARMGAIN_P		  	0.026    				//0.022 - MIDWEST
//#define ARMGAIN_I		  	0.0003					//0.0002 - MIDWEST
//#define ARMGAIN_D		  	0.005	 				//0.005 - MIDWEST

#define	BUTTGAIN_P		  	0.30    				//
#define BUTTGAIN_I		  	0.00000					//
#define BUTTGAIN_D		  	0.001	 				//

//#define ARMUPBIAS			0.00					//DISABLED - 0.05...FEEDFORWARD TO PREVENT DROOP

#define SHOOTERGAIN_P	  	0.0005	    			//0.0015
#define SHOOTERGAIN_I	  	0.0000					//0.00003
#define SHOOTERGAIN_D	  	0.0000	 				//0.0001

#define	LIMEGAIN_P			0.020					//0.015
#define LIMEGAIN_I			0.001					//0.001
#define LIMEGAIN_D			0.00					//0.00

#define GYROGAINH_P		  	0.012					//0.015
#define GYROGAINH_I		  	0.002					//0.001
#define GYROGAINH_D		  	0.00					//0.00

#define GYROGAINL_P		  	0.025					//0.025
#define GYROGAINL_I		  	0.002					//0.002
#define GYROGAINL_D		  	0.00					//0.00

#define ENCODERGAIN_P	  	0.05					//0.05  
#define ENCODERGAIN_I	  	0.001					//0.001   
#define ENCODERGAIN_D	  	0.1						//0.1   




//*********************** SHOOTER *****************************************************

#define MAXSHOOTERSPEEDMAN 	1.0
#define MINSHOOTERSPEEDMAN 	0.0						//0.4 = 2012

#define SHOOTERREVSPEED		-0.2

#define MAXRPM 				9000.0					//MAX RPM attainable with charged battery
#define MINRPM				3000.0					//1500.0 - 2012

//Automatic Speed Commands in RPM for Various Shot Positions
#define	SPEEDOKTHRESH		38						//Defines acceptable +/- tolerance for auto speed control

//Shooter Speed Commands - 2020 SHOOTER


#define RPMSPARESHOT 		4450.0	//OK - Upper Left Click
#define RPMFRONTTRENCH		4450.0	//OK - Left Click
#define RPMFARSHOT			4550.0 	//OK - Lower Left Click

#define RPMPASSMODE			7000.0	//OK - Up Click
#define RPMHOMESHOT			4848.0	//OK - Middle Click
#define RPMTHROUGHTRENCH	0.0	    //OK - Down Click

#define RPMWALLSHOT			5575.0	//OK - Upper Right Click
#define RPMCLIMBMODE		0.0		//OK - Right Click
#define RPMLOWPORT			4000.0	//OK - Lower Right Click

//*********************** LIMELIGHT ****************************************************

#define ONTARGETTHRESH		2		//3		Angular aiming threshold to define whether you are aligned with target




//*********************** MAIN ARM *****************************************************

#define BUMPSPEEDDOWN					25			//Increase/Decrease the value of the current commanded RPM by this amount each time joystick is bumped
#define BUMPSPEEDUP         			25			//Usually 100, increase due to much bigger RPM range

#define UPPERADJUSTDEAD					0.2			//Left gamepad y-axis dead zone for adjustment
#define LOWERADJUSTDEAD	   			   -0.2		    //Left gamepad y-axis dead zone for adjustment

/*
#define MAXARMSPEEDAUTO	 				0.90		//0.90 -
#define MAXARMSPEEDAUTOSLOW				0.70    	//0.70 - 

#define MAXARMSPEEDMAN	     			0.85		//0.75 - 

#define DERATEARMSPEEDUP				1.00		//1.00 - Limit the arm speed within a few degrees of upper boundary
#define DERATEARMSPEEDDOWN				0.80		//0.80 - Limit the arm speed within a few degrees of lower boundary

#define DERATEARMSPEEDDOWNSLOW			0.30		//0.40 - Limit the down arm speed A LOT within a degree or so of the arm lock cylinder
#define DERATEARMSPEEDUPSLOW			0.30		//0.45 - Limit the up arm speed A LOT within a degree or so of the arm lock cylinder 

#define MAXARMSPEEDMANTWEAKUP			0.85		//0.75
#define MAXARMSPEEDMANTWEAKDOWN 		0.85		//0.75

//Semiautomatic arm position adjustments
#define BUMPCMDDOWN						5.0			//Increase/Decrease the value of the current commanded position by this amount each time joystick is bumped
#define BUMPCMDUP           			5.0


//All values in degrees - 90 degrees is straight out from horizontal
//It is OK to command values outside the range if they result in actual values that meet the desired position

//48

#define ARMRANGE					141.0	//Maximum Movement in Degrees

#define ARMUPLIMIT		  			160.0	//160.0 OK	
#define ARMHOMEHIGH					160.0   //160.0 OK    

//LEFT SMART BUTTONS
#define ARMWALLSHOT					85.0    //81.0  OK
#define ARMFRONTTRENCH				25.0	//25.0  OK
#define ARMTHROUGHTRENCH			41.0	//20.0  OK

//MIDDLE SMART BUTTONS
#define ARMPASSMODE      			164.0	//165.0 OK - allow overcommand
#define ARMHOMESHOT       			103.0	//101.0 OK
#define ARMFARSHOT       			50.0    //48.0  OK

//RIGHT SMART BUTTONS
#define ARMCLIMBMODE     			164.0	//160.0 OK
#define ARMSPARESHOT       			105.0	//103.0 OK
#define ARMLOWPORT      			44.0	//42.0  OK

#define ARMDOWNLIMIT				18.0	//20.0  OK	

#define ARMOKTHRESH					2.0			//2.0 - 

#define SLOWARMTHRESH				10.0		//10.0 - Limit arm speed when within this distance of either soft limit
#define SLOWESTARMTHRESH			5.0			//5.0 - Limit arm speed the most within this distance of the lower soft limit
*/

//*********************** CLIMBER *****************************************************

#define	CLIMBEXTENDSPEED		    1.00
#define CLIMBRETRACTSPEED		   -1.00		//Manual control speeds

#define CLIMBEXTENDSAFESPEED        0.25
#define CLIMBRETRACTSAFESPEED	   -0.25

#define DERATECLIMBSPEEDUP			0.75		//0.75 - Limit the climber speed within a few inches of upper boundary
#define DERATECLIMBSPEEDDOWN		0.40		//0.40 - Limit the climber speed within a few inches of lower boundary

//Semiautomatic climber position adjustments
#define CLIMBBUMPCMDUP    			1.0
#define CLIMBBUMPCMDDOWN 			1.0			//Increase/Decrease the value of the current commanded position by this amount each time manual up/down buttons are pressed


//48

#define CLIMBERRANGE			31.5	//Maximum Movement in Inches

#define CLIMBERUPLIMIT		  	29.5	

#define CLIMBERMAX             	29.5

#define CLIMBERDOWNLIMIT		5.0

#define CLIMBERHOME			 	1.0

#define CLIMBEROKTHRESH			1.0		//
#define SLOWCLIMBERTHRESHUP		2.0		//Limit climber speed when within this distance of upper soft limit
#define SLOWCLIMBERTHRESHDOWN	0.0		//Limit climber speed when within this distance of lower soft limit

//*********************** POWER CELL INTAKE *****************************************

#define SUCKSPEED	     	-0.9	//-1.00 - 2021 NEED TO DEFINE NEW STATE FOR SHOOTER FEED SPEED
#define HOLDSPEED        	-0.30	//
#define SPITSPEED	         1.00	//

//*********************** BALL FEED DRIVE *****************************************************

#define BALLFEEDSHOOTSPEED		 0.7

#define BALLFEEDLOADSPEED   	 1.0
#define BALLFEEDLOADSPEEDSLOW    0.5

#define BALLFEEDEJECTSPEED		-1.0
#define BALLFEEDEJECTSPEEDSLOW  -0.5

//*********************** BALL FEED DRIVE *****************************************************

#define R2D2SPEED				 0.35		//ROUGHLY 35% output for 60 color wheel RPM


//*********************** DRIVETRAIN *******************************************************


#define NORMDRIVESPEEDLEFT				0.90			//0.70  DEBUG 3-13-2021 (USUALLY 1.0 FOR EACH)
#define NORMDRIVESPEEDRIGHT				0.90			//0.70
// ^^ These are superseded by MAXSPEED set by adjustment throttle on left pilot stick

#define NORMDRIVESPEEDMIN				0.60			//60% minimum speed
#define NORMDRIVESPEEDMAX				1.00			//100% maximum speed

#define DRIVESTRAIGHTMAXCORRECT			0.25		   	//0.25 
#define SLOWRATE 						0.70

//#define TURBODRIVESPEEDLEFT			1.0
//#define TURBODRIVESPEEDRIGHT			0.97


//*********************** BLINKIN LED *****************************************************


//BLINKINCODE LED LIBRARY

	#define RAINBOWDOUBLE		-0.99
	#define RAINBOWPARTY		-0.97
	#define RAINBOWOCEAN		-0.95
	#define RAINBOWLAVA			-0.93
	#define RAINBOWFOREST		-0.91
	#define RAINBOWGLITTER		-0.89
	#define CONFETTI			-0.87
	#define REDSHOT				-0.85
	#define BLUESHOT			-0.83
	#define WHITESHOT			-0.81
	#define SINELONRAINBOW		-0.79
	#define SINELONPARTY		-0.77
	#define SINELONOCEAN		-0.75
	#define SINELONLAVA 		-0.73
	#define SINELONFOREST		-0.71
	#define BPMRAINBOW			-0.69
	#define BPMPARTY			-0.67
	#define BPMOCEAN			-0.65
	#define BPMLAVA				-0.63
	#define BPMFOREST			-0.61
	#define FIREMEDIUM			-0.59
	#define FIRELARGE			-0.57
	#define TWINKLESRAINBOW		-0.55
	#define TWINKLESPARTY		-0.53
	#define TWINKLESOCEAN		-0.51
	#define TWINKLESLAVA		-0.49
	#define TWINKLESFOREST		-0.47
	#define COLORWAVESRAINBOW	-0.45
	#define COLORWAVESPARTY		-0.43
	#define COLORWAVESOCEAN		-0.41
	#define COLORWAVESLAVA		-0.39
	#define COLORWAVESFOREST	-0.37
	#define LARSONRED			-0.35
	#define LARSONGRAY			-0.33
	#define LIGHTCHASERED		-0.31
	#define LIGHTCHASEBLUE		-0.29
	#define LIGHTCHASEGRAY		-0.27
	#define HEARTBEATRED		-0.25
	#define HEARETBEATBLUE		-0.23
	#define HEARTBEATWHITE		-0.21
	#define HEARTBEATGRAY		-0.19
	#define BREATHRED			-0.17
	#define BREATHBLUE			-0.15
	#define BREATHGRAY			-0.13
	#define STROBERED			-0.11
	#define STROBEBLUE			-0.09
	#define STROBEGOLD			-0.07
	#define STROBEWHITE			-0.05
	#define COLOR1BLACKBLEND	-0.03
	#define COLOR1LARSON		-0.01
	#define COLOR1LIGHTCHASE	 0.01
	#define COLOR1HEARTBEATSLOW	 0.03
	#define COLOR1HEARTBEATMED	 0.05
	#define COLOR1HEARTBEATFAST	 0.07
	#define COLOR1BREATHSLOW	 0.09
	#define COLOR1BREATHFAST	 0.11
	#define COLOR1SHOT			 0.13
	#define COLOR1STROBE		 0.15
	#define COLOR2BLACKBLEND	 0.17
	#define COLOR2LARSON		 0.19
	#define COLOR2LIGHTCHASE	 0.21
	#define COLOR2HEARTBEATSLOW	 0.23
	#define COLOR2HEARTBEATMED	 0.25
	#define COLOR2HEARTBEATFAST	 0.27
	#define COLOR2BREATHSLOW	 0.29
	#define COLOR2BREATHFAST	 0.31
	#define COLOR2SHOT			 0.33
	#define COLOR2STROBE		 0.35
	#define COLOR12SPARKLE		 0.37
	#define COLOR21SPARKLE		 0.39
	#define COLOR12GRADIENT		 0.41
	#define COLOR12BPM			 0.43
	#define COLOR12BLEND		 0.45
	#define COLOR12BLENDEND2END	 0.47
	#define COLOR12NOBLEND		 0.49
	#define COLOR12TWINKLES		 0.51
	#define COLOR12COLORWAVES	 0.53
	#define COLOR12SINELON		 0.55
	#define HOTPINKSOLID		 0.57
	#define DARKREDSOLID		 0.59
	#define REDSOLID			 0.61
	#define REDORANGESOLID		 0.63
	#define ORANGESOLID			 0.65
	#define GOLDSOLID			 0.67
	#define YELLOWSOLID			 0.69
	#define LAWNGREENSOLID		 0.71
	#define LIMEGREENSOLID		 0.73
	#define DARKGREENSOLID		 0.75
	#define GREENSOLID			 0.77
	#define BLUEGREENSOLID		 0.79
	#define AQUASOLID			 0.81
	#define SKYBLUESOLID		 0.83
	#define DARKBLUESOLID		 0.85
	#define BLUESOLID			 0.87
	#define BLUEVIOLETSOLID		 0.89
	#define VIOLETSOLID			 0.91
	#define WHITESOLID			 0.93
	#define GRAYSOLID			 0.95
	#define DARKGRAYSOLID		 0.97
	#define BLACKSOLID			 0.99
	#define BLINKINOFF			 0.99







/*******************************************
 ******************************************* 
 *******************************************
 ******   XM23 CONSTRUCTOR        **********
 *******************************************
 ******************************************* 
 *******************************************/





#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<XM23>(); }
#endif



XM23::XM23()
{


		
	//m_ds = new frc::DriverStation;

	// Object for dealing with the Power Distribution Panel (PDP).
	m_pdp = new frc::PowerDistributionPanel;

	//navX MXP Sensor
	//navX = new AHRS(SerialPort::kMXP); /* Alternatives:  SPI::kMXP, I2C::kMXP or SerialPort::kUSB */

	drive_joystickL = new frc::Joystick(0);		// USB port 0
	drive_joystickR = new frc::Joystick(1);		// USB port 1
	copilot_pad = new LogitechGamepad(2);		// USB port 2
	pilot_pad = new LogitechGamepad(3);  		// USB port 3


/*  roboRIO CAN Device Outputs (00-99)

	   00: PDP and PCM 								(N/A)
	   01: Shooter Top Falcon 500 motor   			(shootertop)
	   02: Shooter Bottom Falcon 500 motor			(shooterbottom)
	   03: Climber Falcon 500 motor					(climber)
	   04:
	   05:
	   06:
	   07:
	   08:
	   09:
	   10: 
	   11:
	   12:
	   13:
	   14:
	   15:
	   16:
	   17:
	   18:
	   19:
	   20:
	   21:
	   22:
	   23:
	   24:
	   25:
	   26:
	   27:
	   28:
	   29:
	   30:
	   31:
	   32:
	   33:
	   34:
	   35:
	   36:
	   37:
	   38:
	   39:
	   40:
	   41:
	   42:
	   43:
	   44:
	   45:
	   46:
	   47:
	   48:
	   49:
	   50:
	   51:
	   52:
	   53:
	   54:
	   55:
	   56:
	   57:
	   58:
	   59:
	   60:
	   61:
	   62:
	   63:
	   64:
	   65:
	   66:
	   67:
	   68:
	   69:
	   70:
	   71:
	   72:
	   73:
	   74:
	   75:
	   76:
	   77:
	   78:
	   79:
	   80:
	   81:
	   82:
	   83:
	   84:
	   85:
	   86:
	   87:
	   88:
	   89:
	   90:
	   91:
	   92:
	   93:
	   94:
	   95:
	   96:
	   97:
	   98:
	   99:
	
*/

	shootertop = 			new WPI_TalonFX(1);															// CAN 01
	shooterbottom = 		new WPI_TalonFX(2);															// CAN 02   
	climber = 				new WPI_TalonFX(3);															// CAN 03
	ballfeed = 				new rev::CANSparkMax(10, rev::CANSparkMax::MotorType::kBrushless);		// CAN 10
	//r2d2 =				new rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless);		// CAN 11


/*	roboRIO PWM/Servo Outputs
		00: Right Front Drive Motor	 			(driveRF)
		01: Right Rear Drive Motor 				(driveRR)
		02: Left Rear Drive Motor				(driveLR)
		03: Left Front Drive Motor	 			(driveLF)
		04: Right Mid Drive Motor				(driveRMID)
		05: Left Mid Drive Motor				(driveLMID)
		06: Power Cell Intake					(intake)
		07: Blinkin LED Module					(blinkin)
		08: Spare Motor							(spare)
		09: Spare Motor							(spare)
		MXP I/O ---------------------------------------------------
		10: Spare Motor							(spare)								
		11: Spare Motor			 				(spare)
		15: Spare Motor							(spare)

*/


	//48 - COMP BOT
	
	driveRF = 			new rev::SparkMax(0);		// PWM 0 - RED
	driveRR = 			new rev::SparkMax(1);		// PWM 1 - ORANGE
	driveLR = 			new rev::SparkMax(2); 		// PWM 2 - PURPLE
	driveLF = 			new rev::SparkMax(3); 		// PWM 3 - BLUE
	driveRMID =			new rev::SparkMax(4); 		// PWM 4 - GRAY
	driveLMID =			new rev::SparkMax(5); 		// PWM 5 - WHITE	
	intake = 			new rev::SparkMax(6);		// PWM 6 - GREEN					
	blinkin =			new frc::VictorSP(7);		// PWM 7 - YELLOW
	//spare =			new frc::VictorSP(8);		// PWM 8 - BROWN
    //spare =    		new frc::VictorSP(9);		// PWM 9 - BLACK
	//--------------------------------------------------------------
	//spare =		    new frc::VictorSP(10);		// PWM 10 - RED/WHITE
	//spare = 			new frc::VictorSP(11);		// PWM 11 - ORANGE/WHITE
	//spare =			new frc::VictorSP(15);		// PWM 15 - WHITE/WHITE



/*
	roboRIO Digital I/O
		00: Right Drive Encoder Channel A 		(RightEncoderA) - RED
		01: Right Drive Encoder Channel B 		(RightEncoderB) - ORANGE
		02: Spare								(spare) - PURPLE
		03: Spare				     			(spare) - BLUE
		04  Spare								(spare) - GRAY
		05: Spare							    (spare) - WHITE
		06: Spare								(spare) - GREEN
		07: Spare								(spare) - YELLOW
		08: Cargo Present Diffuse Photoeye     	(spare) - BROWN
		09: Spare								(spare) - BLACK
		MXP I/O ----------------------------------------------------------------
		13: Spare								(spare) 
		15: Spare								(spare) 
		17: Spare								(spare)
		18: Spare								(spare)

	roboRIO Relay Outputs
		00: Aiming Flashlight					(light)
		01:
		02:
		03:

*/
	
	//US Digital S4 Encoder - A-channel on digital I/O 0, B on digital I/O 1
	RightEncoder = new AugmentedEncoder(0, 1, distance_per_rev / ticks_per_rev, false);

	//VEXPro Shooter Encoders
	//ShooterEncoder = new AugmentedEncoder(4, 5, 1.0 / tticks_per_rev, true);

	//VEXPro Encoders
	//ArmEncoder = new AugmentedEncoder(2, 3, adistance_per_rev / lticks_per_rev, true);

	//CAN Encoders (REV and VEX/CTRE)

	//R2D2Encoder 		 = new rev::CANEncoder(*r2d2);
	BallFeedEncoder 	 = new rev::CANEncoder(*ballfeed);

	//Digital Sensors

	//ballsensorlow		 = new frc::DigitalInput(2);
	//ballsensorhigh     = new frc::DigitalInput(3);
	armdownlimit   	 	 = new frc::DigitalInput(4);
	armuplimit		 	 = new frc::DigitalInput(5);
	climbretractlimit 	 = new frc::DigitalInput(6);
	climbextendlimit	 = new frc::DigitalInput(7);

	

	//light = new frc::Relay(0, frc::Relay::kForwardOnly);

	
	//Compressor now connected to 2015+ PCM (CAN ID 0)
	airpump = new frc::Compressor(0);
	
/*
	roboRIO Analog Inputs

		00: Gyrochip							(gyro) - RED
		01: Low Ball Present Sensor				(balllowsens) - ORANGE
		02: High Ball Present Sensor			(ballhighsens) - PURPLE
		03: spare								(spare) - BLUE

*/

	gyro = new frc::ADXRS450_Gyro();
	// gyro = new frc::ADIS16470_IMU();

	#define AUTOMATIONDIRECTPRESENTTHRESH   1.07	//This is the middle voltage between low and high sensor triggers - AD sensors

	//gyro = new frc::AnalogGyro(0); 			// Analog input 0
	balllowsens = new frc::AnalogInput(1);  	// Analog input 1
	ballhighsens = new frc::AnalogInput(2);		// Analog input 2
	//sparesens = new frc::AnalogInput(3);		// Analog input 3

/*
	roboRIO Solenoid Ouputs (PCM #1 - CAN ID 0)

		LOW PRESSURE MANIFOLD:
		00: Gearbox Shifter (LOWGEAR)				(shifter) - RED
		
		HIGH PRESSURE MANIFOLD:	
		01: Shooter Hood (SHOOTCLOSE)				(hood) - ORANGE
		02: Shooter Cage Tilt (TILTFORWARD)			(tilt) - PURPLE
		03: Shooter Cage Tllt (TILTBACK)			(tilt) - BLUE
		04: Intake Deploy (INTAKEOUT)				(intakedeploy) - GRAY
		05: Intake Deploy (INTAKEIN)				(intakedeploy) - WHITE
		06: Climber Lock  (LOCKON)					(climblock) - GREEN
		07: Climber Lock  (LOCKOFF)					(climblock) - YELLOW
		
		
*/

	shifter = new frc::Solenoid(0,0);
	hood = new frc::Solenoid(0,1);
	tilt = new DoubleSolenoid48 (0,2,0,3);
	intakedeploy = new DoubleSolenoid48 (0,4,0,5);
	climblock = new DoubleSolenoid48 (0,6,0,7);


/*
	roboRIO Solenoid Ouputs (PCM #2 - CAN ID 1)
	
		00: Spare  									(spare)
		01: Spare									(spare)
		02: Spare  									(spare)
		03: Spare									(spare)
		04: Spare  									(spare)
		05: Spare									(spare)
		06: Spare  									(spare)
		07: Spare									(spare)

*/
			
	//spare = new DoubleSolenoid48 (1,0,1,1);
	//spare = new DoubleSolenoid48 (1,2,1,3);
	//spare = new DoubleSolenoid48 (1,4,1,5);
	//spare = new DoubleSolenoid48 (1,6,1,7);
		
	//Driver Station LCD
	//dsLCD = frc::DriverStationLCD::GetInstance();   //disabled for 2015
	
	//InsightLT Display
	//insight = new InsightLT(FOUR_ZONES);
	//insight.startDisplay();
	
	/*
	 *   disp_userInfo.setHeader("Mode:");
  	  	  disp_numberOfGamePieces.setHeader("Items"):
  	  	  display.registerData(disp_userInfo, 1);
  	  	  display.registerData(disp_numberOfGamePieces);
  	  	  display.startDisplay();
	 */
	
	//Timers
	autotimer = new frc::Timer();
	delaytimer = new frc::Timer();
	shifttimer = new frc::Timer();
	cargoholdtimer = new frc::Timer();
	blinktimer = new frc::Timer();
	blinkintimer = new frc::Timer();

	// Set up PID controls.
	EncoderYCtrl = new SimPID(0.0, 0.0, 0.0, 0);
	GyroXCtrl = new SimPID(0.0, 0.0, 0.0, 0);
	//armCtrl = new SimPID(0.0, 0.0, 0.0, 0);
	climbCtrl = new SimPID(0.0, 0.0, 0.0, 0);
	shooterCtrl = new SimPID(0.0, 0.0, 0.0, 0);
	
/*******************************
 ****** Initialize Values  *****  
 ******************************/
	
	//SmartButtons
	
	smartbuttonon = 0;
	smartbuttonlock = 0;
				
  	steerlatch = 0; 
  	//armlatch = 0;


  	//armcmd = 0;
  	//adjustlock = 0;
	//armposok = 0;
	upadjustlock = 0;
	autopadrightjoyY = 0.0;

	climbcmd = 0;
	climberadjustlock = 0;
	climberetractadjustlock = 0;
	climbposok = 0;
	climblatch = 0;

	slowlatch = 0;
	
	shootercmd = 0;
	sadjustlock = 0;
	oktoshoot = 0;
	speedok = 0;

	targetdetected = 0;
	targetlocked = 0;
	targetingenabled = 0;

	ballpresentlow = 0;
	ballpresenthigh = 0;

	ballholdlatch = 0;

	//armatupper = 0;
	//armatlower = 0;
	climbextended = 0;
	climbretracted = 0;

	//Reset all virtual control buttons
	ResetAutoVars();

	SmartReset();
		
	passmode_sw = 0;
	homeshot_sw = 0;
	farshot_sw = 0;
	climbmode_sw = 0;
	spareshot_sw = 0;
	lowport_sw = 0;
	wallshot_sw = 0;
	fronttrench_sw = 0;
	throughtrench_sw = 0;

	autopassmode_sw = 0;
	autohomeshot_sw = 0;
	autofarshot_sw = 0;
	autoclimbmode_sw = 0;
	autospareshot_sw = 0;
	autolowport_sw = 0;
	autowallshot_sw = 0;
	autofronttrench_sw = 0;
	autothroughtrench_sw = 0;

	sbpassmodeactive = 0;
    sbhomeshotactive = 0;
    sbfarshotactive = 0;
    sbclimbmodeactive = 0;
    sbspareshotactive = 0;
    sblowportactive = 0;
    sbwallshotactive = 0;
	sbfronttrenchactive = 0;
	sbthroughtrenchactive = 0;

	//PID
	//armCtrl->setConstants(0.0, 0.0, 0.0);
	//armCtrl->setDesiredValue(0.0);

	climbCtrl->setConstants(0.0, 0.0, 0.0);
	climbCtrl->setDesiredValue(0.0);

	shooterCtrl->setConstants(0.0, 0.0, 0.0);
	shooterCtrl->setDesiredValue(0);

	GyroXCtrl->setConstants(0.0, 0.0, 0.0);
	GyroXCtrl->setDesiredValue(0.0);
	
	EncoderYCtrl->setConstants(0.0, 0.0, 0.0);
	EncoderYCtrl->setDesiredValue(0.0);
	
	// Initialize all motors to neutral.
	// NOTE: If not initialized, they will blink even when enabled.
	
	// Drive System
	DriveNeutral();
	LFdrive.speed = LRdrive.speed = LMIDdrive.speed = RFdrive.speed = RRdrive.speed = RMIDdrive.speed = 0.0;
	maxspeed = 	1.0;	//Default to 100%

	leftx = 0.0;
	lefty = 0.0;
	rightx = 0.0;
	righty = 0.0;
	
	leftxtau = 0.0;
	leftytau = 0.0;
	rightxtau = 0.0;
	rightytau = 0.0;

	leftxcalc = 0.0;
	leftycalc = 0.0;
	rightxcalc = 0.0;
	rightycalc = 0.0;

	drivestate = TANKMODE;
	autodrivestate = DRIVENORMALAUTO;
	controlstate = JOYSTICK;
	
	//Alternate
	//controlstate = GAMEPAD;

	//Gearbox Shifter
	shifter->Set(HIGHGEAR);
	shiftstate = SHIFTHIGH;
	highgearon = 1;
	lowgearon = 0;
	shiftlatch = 0;
	
	//Shooter Systems
	shooterstate = OFF;
	shootermode = AUTOSHOOT;
	shooterspeed = 0.0;
	shootertop->Set(ControlMode::PercentOutput, 0.0);

	//Shooter Hood
	hood->Set(SHOOTFAR);
	hoodstate = HOODUP;

	//Shooter Cage Tilt
	tilt->Set(TILTBACK);
	tiltstate = TILTUP;

	//Intake Deploy
	intakedeploy->Set(INTAKEIN);
	intakedeploystate = RETRACTINTAKE;

	//Climb Lock Cylinder
	climblock->Set(LOCKOFF);
	climblockstate = CLIMBLOCKOFF;

	climblocklatch = 0;
	climblockon = 0;

	//Initialize all non-drive motors
	intakespeed = ballfeedspeed = climbspeed = r2d2speed = 0.0;  //= armspeed = 0.0

	//Arm
	//arm->Set(0);

	//Intake
	intake->Set(0);
	intakestate = INTAKEOFF;

	//Ball Feed
	ballfeed->Set(0);
	ballfeedspeed = 0.0;
	ballfeedstate = BALLFEEDOFF;

	//R2-D2
	//r2d2->Set(0);
	r2d2speed = 0.0;
	r2d2state = R2D2OFF;

	//Aiming Flashlight Relay
	//light->Set(LIGHTOFF);
	
	//Joystick Positions
	joyleftstate = joyrightstate = JOYNEUTRAL;

	// Start the Compressor
	airpump->SetClosedLoopControl(true);

	//airpump->Start(); //should also work 

	// Initialize sensors
	
	//48 - 2020 Disabled
	//gyro->SetSensitivity(0.007); 			// OLD ADXRS: 0.00393  NEW ADXRS: 0.00160  NEW 09 GYRO: 0.?????    DEFAULT - ADXRS300  5.0 mV/deg/sec = .005 V/deg/s, ADXRS150 = 0.0122
	
	gyro->Reset();
	gyroangle = 0.0;
	lastgyroangle = 0.0;
	gyrorate = 0.0;
	gyrojoy = 0.0;
	//xvirt = 0.0;
	//yvirt = 0.0;

	steercorrect = 0.0;

	RightEncoder->Start();
	//ArmEncoder->Start();

	//ShooterEncoder->Start();

	encoderjoy = 0.0;

	auto_periodic_loops = 0;
	disabled_periodic_loops = 0;
	teleop_periodic_loops = 0;

	//SmartButton Initialization
	
	lastsmartstate = SMARTOFF;

	//Default Autonomous Program is Right 2 - Shoot 3, Collect 3 from trench, Shoot 3 more

	autostartpos = 2;   //Autonomous program type: 1 = Left Start, 2 = Center Start, 3 = Right Start
	autoprognum = 2;    //Autonomous program number for starting position: 1-8 (or more)
	
	autostartlock = 0;
	autoproglock = 0;

	autonactive = 0;
	autodriveenable = 1;

}


/*******************************************
 ******************************************* 
 *******************************************
 ******   ROBOT INIT              ********** 
 *******************************************
 ******************************************* 
 *******************************************/

void XM23::RobotInit(void)
{

	//Initialize all CAN Devices
	CANInit();

	//Needed for Basic Analog Devices Gyro ()
	//gyro->Calibrate();

	lefty = leftx = righty = rightx = 0.0;
	leftytau = leftxtau = rightytau = rightxtau = 0.0;
	leftycalc = leftxcalc = rightycalc = rightxcalc = 0.0;
	oldlefty = oldleftx = oldrighty = oldrightx = 0.0;
	LFdrive.speed = LRdrive.speed = LMIDdrive.speed = RFdrive.speed = RRdrive.speed = RMIDdrive.speed = 0.0;
	maxspeed = 	1.0;	//Default to 100%

	DriveNeutral();

	autodrivestate = DRIVENORMALAUTO;
	drivestate = TANKMODE;
	controlstate = JOYSTICK;
	
	//switchstraightstate = RUNSTRAIGHT;

	alliancestate = BLUEALLIANCE;

	gyroangle = 0.0;
	lastgyroangle = 0.0;

	ballvallow = 0.0;
	ballvalhigh = 0.0;
	
	//armspeed = 0.0;
	//armcmd = armpos = 0.0;

	climbspeed = 0.0;
	climbcmd = climbpos = 0.0;

	shooterspeed = 0.0;
	shooterRPM = 0.0;

	r2d2pos = 0.0;

	ballfeedpos = 0.0;

	delayset = 0.0;
	delayadjust = 0.0;

	speedset = 0.0;

	//Camera Initialization

	//Limelight Network Tables
	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
			
	//Limelight LED's OFF
	//DEBUG 2021 - easy indicator that roboRIO code has been restarted
	LightsOn();

	//Set PIP Secondary Stream Mode
	//nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("stream",2);

	// Create and set up a camera instance. first wait for the camera to start
	// if the robot was just powered on. This gives the camera time to boot.
	//printf("Getting camera instance\n");
	
	//AxisCamera &camera = AxisCamera::GetInstance();
	//printf("Setting camera parameters\n");
	
	//camera.WriteResolution(AxisCamera::kResolution_320x240);
	//camera.WriteCompression(20);
	//camera.WriteBrightness(50);
	//the camera name (ex "cam0") can be found through the roborio web interface

	// ENABLE ME FOR USB CAMERAS - 48

	cs::UsbCamera lowcam = frc::CameraServer::GetInstance()->StartAutomaticCapture("LowCam",0);
	//cs::UsbCamera highcam = frc::CameraServer::GetInstance()->StartAutomaticCapture("HighCam",1);

	lowcam.SetResolution(176,144); //(320,240 pulling too much bandwidth for field)
	lowcam.SetFPS(30);
	lowcam.SetExposureAuto();

	//highcam.SetResolution(176,144);
	//highcam.SetFPS(30);
	//highcam.SetExposureAuto();
	
	//ENABLE THE ABOVE FOR USB CAMERAS - 48

    //sink1 = CameraServer::GetInstance()->GetVideo(lowcam);
    //sink2 = CameraServer::GetInstance()->GetVideo(highcam);

    //sink1.SetEnabled(true);
    //sink2.SetEnabled(false);

	//ASSUME ARM AND CLIMBER ARE SET TO STARTING POSITION FOR MATCH AND AUTO START 
	//armoffset = ARMFRONTTRENCH;
	//armishome = 1;

	climber->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
	climboffset = CLIMBERHOME;
	climbishome = 1;
	climberenabled = 0;

	BallFeedEncoder->SetPosition(0.0);

	autonStep = 1;
	gyrodonestore = 0;
	encoderdonestore = 0;

	printf("ROBOT INIT COMPLETE!\n");
}

/*******************************************
 ******************************************* 
 *******************************************
 ******   DISABLED INIT           ********** 
 *******************************************
 ******************************************* 
 *******************************************/

void XM23::DisabledInit(void)
{
	disabled_periodic_loops = 0;

	climberenabled = 0;
	
}


/*******************************************
 ******************************************* 
 *******************************************
 ******   DISABLED PERIODIC       ********** 
 *******************************************
 ******************************************* 
 *******************************************/



void XM23::DisabledPeriodic()
{
	
	if ( (disabled_periodic_loops % 2) == 0)
	{

		DoLimeStuff();

		// put 100Hz Victor control here

		//armpos = ( (ARMRANGE * armsens->GetAverageVoltage() ) / POTRANGE) - armoffset;
		//armpos = armsens->GetAverageVoltage();

		//armpos = ArmEncoder->GetDistance() + armoffset;
		//if (armpos >= ARMDOWNLIMIT + ARMRANGE ) armpos = ARMDOWNLIMIT + ARMRANGE;

		shooterRPM = (shootertop->GetSelectedSensorVelocity(kPIDLoopIdx) * 600 * 30) / (tticks_per_rev * 18);

		climbpos = ( ( (climber->GetSelectedSensorPosition(kPIDLoopIdx) / tticks_per_rev) ) * cdistance_per_rev) + climboffset;

		//r2d2pos = (R2D2Encoder->GetPosition() / rticks_per_rev) * rdistance_per_rev;

		ballfeedpos = (BallFeedEncoder->GetPosition() / rticks_per_rev) * fdistance_per_rev;

		//distanceleft = LeftEncoder->GetDistance();
		distanceright = RightEncoder->GetDistance();

		gyroangle = -gyro->GetAngle();

		//ballpresentlow = 1 - ballsensorlow->Get();
		//ballpresenthigh = 1 - ballsensorhigh->Get();

		GetAnalogSensors();
		
		//armatupper = 1 - armuplimit->Get();
		//armatlower = 1 - armdownlimit->Get();

		climbextended = 1 - climbextendlimit->Get();
		climbretracted = 1 - climbretractlimit->Get();

		maxspeed = NORMDRIVESPEEDMIN + ( (NORMDRIVESPEEDMAX - NORMDRIVESPEEDMIN) * ( (1.0 + speedadjust) / 2.0) );

	}


	/**
	 * Runs periodically while the robot is disabled.
	 */
	
	//Read all controller I/O into Boolean variables	
    GetControllerIO();
	
    //Joystick or Gamepad?
  
    if ( (joystickselect_sw) && (!gamepadselect_sw) )
    {
    	controlstate = JOYSTICK;
    }
    else if ( (gamepadselect_sw) && (!joystickselect_sw) )
    {
    	controlstate = GAMEPAD; 	
    }
        
	//Arcade or tank? 
    
    if ( (arcade_sw) && (!tank_sw) )
    {
    	drivestate = ARCADEMODE;
    }
    else if ( (tank_sw) && (!arcade_sw) )
    {
    	drivestate = TANKMODE;
    }
    
    //Auto-drive (Gyro Assist) or Manual?

    if ( (autodrive_sw) && (!manualdrive_sw) )
    {
    	autodrivestate = DRIVENORMALAUTO;
    }
    else if ( (manualdrive_sw) && (!autodrive_sw) )
	{
    	autodrivestate = DRIVENORMALMAN;
    }

/*
	//Select automatic or manual speed control
	if ( (autoshooter_sw == 1) && (manualshooter_sw == 0) )
	{
		shootermode = AUTOSHOOT;
	}
	else if ( (manualshooter_sw == 1) && (autoshooter_sw == 0) )
	{
		shootermode = MANUALSHOOT;
	}
*/


    // Determine the autonomous start position and program number.
    // 1 = LEFT, 2 = CENTER, 3 = RIGHT
    // Use left stick push for start pos cycle; right stick push for program cycle.

    //starting position
    if (!autopos_sw) autostartlock = 0;
    
    if ( (autopos_sw) && (autostartlock == 0) )
	{
		if (autostartpos < MAXPOS)
		{
			autostartpos++;
		}
		else
		{
			autostartpos = 1;
		}
		
		autostartlock = 1;
	}

    //autonomous program number 
    if (!autoprog_sw) autoproglock = 0;
    
    if ( (autoprog_sw) && (autoproglock == 0) )
	{
		
		if (autoprognum < MAXPROG)
		{
			autoprognum++;
		}
		else
		{
			autoprognum = 2;  //bypass manual control mode - program 1
			//autoprognum = 1;
		}
		
		autoproglock = 1;
	
	}
	
	//Autonomous Program Start Delay
    delayset = ( (1 + delayadjust) / 2 ) * MAXDELAY;

	    
    //printf("delayset: %f, delayadjust = %f\n", delayset, delayadjust);
    //printf("ballpresentlow: %i, goaltapedetected = %i\n", ballpresentlow, goaltapedetected);
	
    //Update the Dashboard

	CustomDashboard();
	
	DoCameraStuff();

	DoLEDStuff();

	disabled_periodic_loops++;
	
	
}


/*******************************************
 ******************************************* 
 *******************************************
 ******   TELEOP INIT             ********** 
 *******************************************
 ******************************************* 
 *******************************************/

void XM23::TeleopInit()
{


	//Get the alliance color and send to dashboard 

	frc::DriverStation::Alliance color;
	color = frc::DriverStation::GetInstance().GetAlliance();

	if (color == frc::DriverStation::Alliance::kBlue)
	{
		alliancestate = BLUEALLIANCE;
		allianceisblue = 1;
		blinkincode = LIGHTCHASEBLUE;
	}
	else if (color == frc::DriverStation::Alliance::kRed) 
	{
		alliancestate = REDALLIANCE;
		allianceisblue = 0;
		blinkincode = LIGHTCHASERED;
	}

	/**
	 * Initializes all motors to neutral to begin teleop.
	 * NOTE: This is important to prevent motors from retaining values from
	 * autonomous.
	 */
	
	autotimer->Stop();
	autotimer->Reset();
	
	blinktimer->Start();
	blinktimer->Reset();

	blinkintimer->Start();
	blinkintimer->Reset();
	
	shifttimer->Start();
	shifttimer->Reset();

	cargoholdtimer->Start();
	cargoholdtimer->Reset();


	SmartReset();			//Reset all Smart Button Positions

	//Reset all virtual control buttons
	ResetAutoVars();
  		
	autonactive = 0;   //IMPORTANT!!
	
	teleop_periodic_loops = 0;
	packets_in_second = 0;

	DriveNeutral();
	
	//Gearbox Shifter
	shifter->Set(HIGHGEAR);
	shiftstate = SHIFTHIGH;
	highgearon = 1;
	lowgearon = 0;
	shiftlatch = 0;
	
	//Shooter Hood
	hood->Set(SHOOTFAR);
	hoodstate = HOODUP;

	//Shooter Cage Tllt
	tilt->Set(TILTBACK);
	tiltstate = TILTUP;

	slowlatch = 0;
	
	//Climb Lock Cylinder
	climblock->Set(LOCKOFF);
	climblockstate = CLIMBLOCKOFF;

	//Shooter Systems
	shooterstate = OFF;
	shootermode = AUTOSHOOT;

	shooterspeed = 0.0;
	shootertop->Set(ControlMode::PercentOutput, 0.0);

	//Intake System
	intake->Set(0);
	intakespeed = 0.0;
	intakestate = INTAKEOFF;

	//Ball Feed System
	ballfeed->Set(0);
	ballfeedspeed = 0.0;
	ballfeedstate = BALLFEEDOFF;

	//R2-D2
	//r2d2->Set(0);
	r2d2speed = 0.0;
	r2d2state = R2D2OFF;

	//Climber System
	climber->Set(ControlMode::PercentOutput, 0);
	climbspeed = 0.0;
	climbstate = CLIMBOFF;

	//Joystick Positions
	joyleftstate = joyrightstate = JOYNEUTRAL;
	
	//Reset the gyro
	gyro->Reset();
	gyroangle = 0.0;
	lastgyroangle = 0.0;
	gyrorate = 0.0;
	gyrojoy = 0.0;
	encoderjoy = 0.0;
	//xvirt = 0.0;
	//yvirt = 0.0;
	
	steercorrect = 0.0;

	//Reset the drivetrain encoder(s)

	RightEncoder->Reset();

	//Reset the gyro
	gyro->Reset();

	//Aiming Light
	//light->Set(LIGHTOFF);
	
	//Turn the limelight lights and shooter off
	autoshooteroff_sw = 1; 
	LightsOff();


}


/*******************************************
 ******************************************* 
 *******************************************
 ******   TELEOP PERIODIC         ********** 
 *******************************************
 ******************************************* 
 *******************************************/

void XM23::TeleopPeriodic()
{
	
	/**
	 * Runs periodically during teleoperated mode.
	 * Maps joystick and gamepad input to motor output. 
	 */
	
	teleop_periodic_loops++;

	/*******************************
	 ******************************* 
	 *******************************
	 ******   50 Hz       ********** 
	 *******************************
	 ******************************* 
	 ******************************/
	
	if ((teleop_periodic_loops % 4) == 0) 
	{
		// put 50Hz servo control here
		
		//Write information to the dashboard

		//Get Random Color for Stage 3 Color Wheel
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if(gameData.length() > 0)
		{

			if (gameData[0] == 'R')
			{
				colorstate = CP_RED;
			}
			else if (gameData[0] == 'G')
			{
				colorstate = CP_GREEN;
			}
			else if (gameData[0] == 'B')
			{
				colorstate = CP_BLUE;
			}
			else if (gameData[0] == 'Y')
			{
				colorstate = CP_YELLOW;
			}				
			else  //FIRST done f'd up
			{
				colorstate = CP_ERROR;
			}

		}
		else
		{
			colorstate = CP_ERROR;
		}

		CustomDashboard();

		//shooterRPM was originally calculated here
	    
	}
	
	/*******************************
	 ******************************* 
	 *******************************
	 ******   100 Hz      ********** 
	 *******************************
	 ******************************* 
	 ******************************/
	
	if ((teleop_periodic_loops % 2) == 0) 
	{
			
		// put 100Hz Victor control here

		DoLimeStuff();

		//Calculate the arm angle
		//armpos = ( (ARMRANGE * armsens->GetAverageVoltage() ) / POTRANGE) - armoffset;
		//armpos = armsens->GetAverageVoltage();

		//armpos = ArmEncoder->GetDistance() + armoffset;
		//if (armpos >= ARMDOWNLIMIT + ARMRANGE) armpos = ARMDOWNLIMIT + ARMRANGE;

		//Cherry Angle Sensor 2013-14
		//armpos = 180 - ((int) (180.0 - (180.0 * ( (armsens->GetAverageVoltage() - 0.5) / 4.0)) - armoffset));
		//Cherry Angle Sensor 2011-2012
		//armpos = (int) (180.0 - (180.0 * ( (armsens->GetAverageVoltage() - 0.5) / 4.0)) - armoffset);
		//rangepos = (int) rangesens->GetAverageVoltage();

		//Calculate the shooter RPM	for the shooter wheel

		shooterRPM = (shootertop->GetSelectedSensorVelocity(kPIDLoopIdx) * 600 * 30) / (tticks_per_rev * 18);

		climbpos = ( ( (climber->GetSelectedSensorPosition(kPIDLoopIdx) / tticks_per_rev) ) * cdistance_per_rev) + climboffset;

		//r2d2pos = (R2D2Encoder->GetPosition() / rticks_per_rev) * rdistance_per_rev;

		ballfeedpos = (BallFeedEncoder->GetPosition() / rticks_per_rev) * fdistance_per_rev;

		maxspeed = NORMDRIVESPEEDMIN + ( (NORMDRIVESPEEDMAX - NORMDRIVESPEEDMIN) * ( (1.0 + speedadjust) / 2.0) );

		//2017
		//ShooterEncoder->Recalculate();
		//shooterRPM =  60.0 * ShooterEncoder->GetVelocity();	//velocity in RPM (GetVelocity() returns revs/sec)
		
		//2013
		//shooterRPM = (int) ( 60.0 / ShooterEncoder->GetPeriod());

		//printf("cmd: %i, pos: %i, speed: %f, lenc: %f, renc: %f, gyro: %f\n", armcmd, armpos, armspeed, distanceleft, distanceright, gyroangle);
		//printf("gyro: %f, gyrojoy: %f\n, enc: %f, encjoy: %f, ls: %f, rs: %f\n", gyroangle, gyrojoy, distanceright, encoderjoy, LFdrive.speed, RFdrive.speed);
	    //printf("joyY: %f, joyX: %f\n", padleftjoyY, padleftjoyX);
		//printf("Rspeed: %f\n", shooterRPM);

		//Read in the gyro angle
		
		gyroangle = -gyro->GetAngle();
				
		//gyrorate = gyro->m_analog->GetAverageBits();  //Only with Gyro48
		//printf("D gyro  angle: %f, gyro rate: %f\n", gyroangle, gyrorate);
				
		//printf("Arm height: %f\n", armpos);
		
		//distanceleft = LeftEncoder->GetDistance();
		distanceright = RightEncoder->GetDistance();
	
		//printf("cmd: %i, pos: %f, speed: %f, enc: %f, gyro: %f\n", armcmd, armpos, armspeed, distanceright, gyroangle);
		//printf("arm: %i, len: %f, ren: %f, gyr: %f, speed: %i\n", armpos, distanceleft, distanceright, gyroangle, shooterRPM);
		//printf("cmd: %f, pos: %f, speed: %f, enc: %f, gyro: %f\n", armcmd, armpos, armspeed, distanceright, gyroangle);

		//ballpresentlow = 1 - ballsensorlow->Get();
		//ballpresenthigh = 1 - ballsensorhigh->Get();

		GetAnalogSensors();

		//armatupper = 1 - armuplimit->Get();
		//armatlower = 1 - armdownlimit->Get();

		climbextended = 1 - climbextendlimit->Get();
		climbretracted = 1 - climbretractlimit->Get();

		//Auto-drive (Gyro Assist) or Manual? -- MUST be active in teleop for safety reasons (i.e. Gyro breaks)
		if ( (autodrive_sw) && (!manualdrive_sw) )
		{
			autodrivestate = DRIVENORMALAUTO;
		}
		else if ( (manualdrive_sw) && (!autodrive_sw) )
		{
			autodrivestate = DRIVENORMALMAN;
		}

		//MOVED TO 100 HZ LOOP 2020 - TEST
		//Read all controller I/O into Boolean variables	
    	
		GetControllerIO();	
		
	}		

	
	/*******************************
	 ******************************* 
	 *******************************
	 ******   200 Hz      ********** 
	 *******************************
	 ******************************* 
	 ******************************/

	//Do that funky drive stuff, white boy.
	if (controlstate == JOYSTICK)
	{
		DoDriveStuff(leftjoyY, leftjoyX, rightjoyY, rightjoyX);
	}
	else //controlstate == GAMEPAD
	{
		DoDriveStuff(padleftjoyY, padleftjoyX, padrightjoyY, padrightjoyX);		
	}
	
	DoShiftStuff();
	
  	DoSmartStuff();			// Smart Button administration code

	DoCANStuff();
  	
	//DoArmStuff();

	DoClimbStuff();

	DoShooterStuff();

	DoOtherStuff();
	
	DoCameraStuff();

	DoLEDStuff();

	BlinkControl();
	
	//TELEOP PRINTF DU JOUR

	//printf("tleft = %i, tright = %i\n", (int) ballpresentlow, (int) jawsareopen);
	//printf("blink = %f\n", blinktimer->Get());
	
	
}




/*******************************************
 ******************************************* 
 *******************************************
 ******   AUTONOMOUS INIT         ********** 
 *******************************************
 ******************************************* 
 *******************************************/


void XM23::AutonomousInit()
{

	//Get the alliance color and send to dashboard 

	frc::DriverStation::Alliance color;
	color = frc::DriverStation::GetInstance().GetAlliance();

	if (color == frc::DriverStation::Alliance::kBlue)
	{
		alliancestate = BLUEALLIANCE;
		allianceisblue = 1;
		blinkincode = BLUESHOT;
	}
	else if (color == frc::DriverStation::Alliance::kRed) 
	{
		alliancestate = REDALLIANCE;
		allianceisblue = 0;
		blinkincode = REDSHOT;
	}

	//autoSelected = *((std::string*)chooser->GetSelected());
	//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
	//std::cout << "Auto selected: " << autoSelected << std::endl;

	//if(autoSelected == autoNameCustom){
		//Custom Auto goes here
	//} else {
		//Default Auto goes here
	//}
	
	/**
	 * Initializes autonomous mode settings at the beginning of autonomous.
	 */
	
	SmartReset();
	
	//Reset all virtual control buttons
	ResetAutoVars();
	
	auto_periodic_loops = 0;
	
 	autonStep = 1;
	gyrodonestore = 0;
	encoderdonestore = 0;
 	
	gyro->Reset();
	gyroangle = 0.0;
	lastgyroangle = 0.0;
	gyrorate = 0.0;
		
	autotimer->Start();
	delaytimer->Start();
	shifttimer->Start();
	cargoholdtimer->Start();
	blinktimer->Start();
	blinkintimer->Start();
	
	autotimer->Reset();
	delaytimer->Reset();
	shifttimer->Reset();
	cargoholdtimer->Reset();
	blinktimer->Reset();
	blinkintimer->Reset();

	distanceleft = 0.0;
	distanceright = 0.0;
	
	RightEncoder->Reset();

	EncoderYCtrl->resetErrorSum();
	GyroXCtrl->resetErrorSum();
	//armCtrl->resetErrorSum();
	climbCtrl->resetErrorSum();
	shooterCtrl->resetErrorSum();
	
	EncoderYCtrl->setConstants(0.0, 0.0, 0.0);
	GyroXCtrl->setConstants(0.0, 0.0, 0.0);
	//armCtrl->setConstants(0.0, 0.0, 0.0);
	climbCtrl->setConstants(0.0, 0.0, 0.0);
	shooterCtrl->setConstants(0.0, 0.0, 0.0);
	
	DriveNeutral();
	
	//Gearbox Shifter
	shifter->Set(LOWGEAR);
	shiftstate = SHIFTLOW;
	highgearon = 0;
	lowgearon = 1;
	shiftlatch = 0;

	//Shooter Hood
	hood->Set(SHOOTFAR);
	hoodstate = HOODUP;

	//Cargo Tllt
	tilt->Set(TILTBACK);
	tiltstate = TILTUP;

	slowlatch = 0;
	
	//Spare Gripper
	climblock->Set(LOCKOFF);
	climblockstate = CLIMBLOCKOFF;
	
	//Aiming Light
	//light->Set(LIGHTOFF);
	
	//ASSUME ARM AND MONKEY ARM ARE SET TO STARTING POSITION FOR MATCH AND AUTO START 
	//armoffset = ARMFRONTTRENCH;
	//armishome = 1;

	climboffset = CLIMBERHOME;
	climbishome = 1;
	climberenabled = 0;

	
	/*
	//NEW FOR 2018!!! - drive autoprognum based on the status of autostartpos, scalestate, and switchstate
	if (autoprognum <= 6) //OK to run game data-driven auto modes if human user hasn't selected drive forward only or do nothing
	{

		if (autostartpos == 1)  //We're starting on the left side
		{

			if ( (switchstate == LEFTSWITCH) && (scalestate == RIGHTSCALE) )
			{

				if (switchstraightstate == RUNSWITCH)
				{
					autoprognum = 1;  //Run AutoLeft1 - Left Switch x2
				}
				else
				{
					autoprognum = 6;  //Run AutoLeft6 - Left Switch from Rear Left Side (Defer to Center Start Robot)
					//autoprognum = 5; 	//Run AutoLeft5 - Starting on left, cross field for right scale
				}

			}
			else if ( (switchstate == LEFTSWITCH) && (scalestate == LEFTSCALE) )
			{
				autoprognum = 2;  //Run AutoLeft2 - Left Scale and Switch
			}
			else if ( (switchstate == RIGHTSWITCH) && (scalestate == LEFTSCALE) )
			{
				autoprognum = 3;  //Run AutoLeft3 - Left Scale x2
			}
			else if ( (switchstate == RIGHTSWITCH) && (scalestate == RIGHTSCALE) )
			{
				//autoprognum = 5;  //Run AutoLeft5 - Right Scale then collect
				autoprognum = 7;	//Run AutoLeft7 - Drive Forward Only - CROSS FIELD DISABLED
			}
			else if ( (switchstate == SWITCHERROR) || (scalestate == SCALEERROR) )
			{
				autoprognum = 7;  //Run AutoLeft7 - Drive Forward Only
			}

		}
		else if (autostartpos == 2) //We're starting in the center
		{

			if (switchstate == LEFTSWITCH)
			{
				autoprognum = 1;  //Run AutoCenter1 - Left Switch x2
			}
			else if (switchstate == RIGHTSWITCH)
			{
				autoprognum = 2;  //Run AutoCenter2 - Right Switch x2
			}
			else if (switchstate == SWITCHERROR)
			{
				autoprognum = 7;  //Run AutoCenter7 - Drive Forward Only
			}

		}
		else if (autostartpos == 3) //We're starting on the right side
		{

			if ( (switchstate == RIGHTSWITCH) && (scalestate == LEFTSCALE) )
			{
				if (switchstraightstate == RUNSWITCH)
				{
					autoprognum = 1;  //Run AutoRight1 - Left Switch x2
				}
				else
				{
					autoprognum = 6;  //Run AutoRight6 - Right Switch from Rear Right Side (Defer to Center Start Robot)
					//autoprognum = 5; 	//Run AutoRight5 - Starting on right, cross field for left scale
				}

			}
			else if ( (switchstate == RIGHTSWITCH) && (scalestate == RIGHTSCALE) )
			{
				autoprognum = 2;  //Run AutoRight2 - Right Scale and Switch
			}
			else if ( (switchstate == LEFTSWITCH) && (scalestate == RIGHTSCALE) )
			{
				autoprognum = 3;  //Run AutoRight3 - Right Scale x2
			}
			else if ( (switchstate == LEFTSWITCH) && (scalestate == LEFTSCALE) )
			{
				//autoprognum = 5;  //Run AutoRight5 - Left Scale then collect
				autoprognum = 7;	//Run AutoRight7 - Drive Forward Only - CROSS FIELD DISABLED
			}
			else if ( (switchstate == SWITCHERROR) || (scalestate == SCALEERROR) )
			{
				autoprognum = 7;  //Run AutoRight7 - Drive Forward Only
			}

		}

	}

	*/

}




/*******************************************
 ******************************************* 
 *******************************************
 ******   AUTONOMOUS PERIODIC     ********** 
 *******************************************
 ******************************************* 
 *******************************************/

void XM23::AutonomousPeriodic(void)
{
	/**
	 * Runs periodically during autonomous mode.
	 * Uses sensor inputs and PID to control motor outputs directly.
	 */

	autonactive = 1;   //IMPORTANT!!!
	
	auto_periodic_loops++;

		
	//50 Hz 
	if ((auto_periodic_loops % 4) == 0) 
	{

		CustomDashboard();

	}
	
	//100 Hz
	if ((auto_periodic_loops % 2) == 0) 
	{	

		DoLimeStuff();

		//Read in the arm potentiometer value and scale
		//armpos = ( (ARMRANGE * armsens->GetAverageVoltage() ) / POTRANGE) - armoffset;

		//armpos = ArmEncoder->GetDistance() + armoffset;

		//if (armpos >= ARMDOWNLIMIT + ARMRANGE) armpos = ARMDOWNLIMIT + ARMRANGE;

		//Cherry Angle Sensor 2013-14
		//armpos = 180 - ((int) (180.0 - (180.0 * ( (armsens->GetAverageVoltage() - 0.5) / 4.0)) - armoffset));
		
		shooterRPM = (shootertop->GetSelectedSensorVelocity(kPIDLoopIdx) * 600 * 30) / (tticks_per_rev * 18);

		climbpos = ( ( (climber->GetSelectedSensorPosition(kPIDLoopIdx) / tticks_per_rev) ) * cdistance_per_rev) + climboffset;

		//r2d2pos = (R2D2Encoder->GetPosition() / rticks_per_rev) * rdistance_per_rev;

		ballfeedpos = (BallFeedEncoder->GetPosition() / rticks_per_rev) * fdistance_per_rev;

		//2017
		//ShooterEncoder->Recalculate();
		//shooterRPM =  60.0 * ShooterEncoder->GetVelocity();	//velocity in RPM (GetVelocity() returns revs/sec)
		
		//2013
		//shooterRPM = (int) ( 60.0 / ShooterEncoder->GetPeriod());

		//Read in the gyro angle
		gyroangle = -gyro->GetAngle();
		
		//Read in the encoder value(s)
		//distanceleft = LeftEncoder->GetDistance();
		distanceright = RightEncoder->GetDistance();
		//printf("Encoder: %f\n", distanceright);

		//ballpresentlow = 1 - ballsensorlow->Get();
		//ballpresenthigh = 1 - ballsensorhigh->Get();

		GetAnalogSensors();
		
		//armatupper = 1 - armuplimit->Get();
		//armatlower = 1 - armdownlimit->Get();

		climbextended = 1 - climbextendlimit->Get();
		climbretracted = 1 - climbretractlimit->Get();

		if ( (autodrive_sw) && (!manualdrive_sw) )
		{
			autodrivestate = DRIVENORMALAUTO;
		}
		else if ( (manualdrive_sw) && (!autodrive_sw) )
		{
			autodrivestate = DRIVENORMALMAN;
		}

		//MOVED TO 100 HZ LOOP 2020 
		//Read all controller I/O into Boolean variables	
    	GetControllerIO();	

		//Do that funky drive stuff, white boy.
		if (controlstate == JOYSTICK)
		{
			DoDriveStuff(leftjoyY, leftjoyX, rightjoyY, rightjoyX);
		}
		else //controlstate == GAMEPAD
		{
			DoDriveStuff(padleftjoyY, padleftjoyX, padrightjoyY, padrightjoyX);		
		}
	
	}
	
	/*
		if (auto_periodic_loops == 1) {
			//start doing something
		}
		if (auto_periodic_loops == (2 * GetLoopsPerSec())) {
			//do something else after two seconds
		}
	*/

	/*
	ASSUME ALL PROGRAMS START WITH 3 POWER CELLS
	ASSUME ALL PROGRAMS START FACING SCORING PORT

	PROGRAM LEFT:

		{Turn 90 right
		Back up x 
		Turn 90 left}
	{Arc turn?}
		Lower Arm?
		Turn on intake
		Back up X
		Turn 45 right
		Drive forward x & raise arm & turn off intake
		Turn 45 left & spin up shooter
		Fire x5
	If enough time left, continue ------
		Turn 90 left
		Back up x 
		Turn 90 right
		Turn on intake
		Back up x 
		Drive forward to front of trench & turn off intake & spin up shooter
		Turn left towards goal
		Fire x3

	PROGRAM CENTER:

		Turn towards goal & spin up shooter
		Fire x3
		Turn opposite previous turn
		Back up x 
		Turn 90 left
		Back up x 
		Turn 90 right
		Turn on intake
		Back up x 
		Drive forward to front of trench & turn off intake & spin up shooter
		Turn left towards goal
		Fire x3

	PROGRAM RIGHT:

		Turn towards goal (if needed) & spin up shooter
		Fire x3
		Turn left (90 - previous turn)
		Back up x 
		Turn 90 right
		Turn on intake
		Back up x 
		Drive forward to front of trench & turn off intake & spin up shooter
		Turn left towards goal
		Fire x5
	*/


	/*

	AUTONOMOUS ROUTINES SUMMARY

	2020 Programs - Left Alliance Starting Position
	Left 1 - Full Manual Control
	Left 2 - Default - Pick up 2 Balls from Opponent Trench, Shoot 5
	Left 3 - 
	Left 4 - 
	Left 5 - 
	Left 6 - 
	Left 7 - Drive Forward Only
	Left 8 - Do Nothing

	2020 Programs - Center Alliance Starting Position
	Center 1 - Full Manual Control
	Center 2 - Default - Shoot 3 Balls, Move off Initiation Line
	Center 3 - 
	Center 4 - 2021 AUTONAV CHALLENGE - LOW GEAR BOUNCE PATH (BALLS)
	Center 5 - 2021 AUTONAV CHALLENGE - LOW GEAR SLALOM PATH (FISH)
	Center 6 - 
	Center 7 - Drive Forward Only
	Center 8 - Do Nothing

	2020 Programs - Right Alliance Starting Position
	Right 1 - Full Manual Control
	Right 2 - Default - Shoot 3 Balls, Gather 3 from Trench, Shoot 3
	Right 3 - 
	Right 4 - 2021 AUTONAV CHALLENGE - HIGH GEAR BOUNCE PATH (BALLS)
	Right 5 - 2021 AUTONAV CHALLENGE - HIGH GEAR SLALOM PATH (FISH)
	Right 6 - 2021 AUTONAV CHALLENGE - HIGH GEAR BARREL RACING PATH
	Right 7 - Drive Forward Only
	Right 8 - Do Nothing
	
	*/


	//The program select code below remains unchanged from previous years - we are just inserting additional
	//decision-making code in AutonomousInit to write autoprognum based on the start position and the "LLL" game data
	
	switch (autostartpos)
	{
		case 1: //2020 Left Alliance Programs
		{
			switch (autoprognum)
			{
				case 1:
				{	
					//AutoLeft1();
					break;
				}
				case 2:
				{
					//AutoLeft2();
					break;
				}
				case 3:
				{
					//AutoLeft3();
					break;
				}
				case 4:
				{
					//AutoLeft4();
					break;
				}
				case 5:
				{
					//AutoLeft5();
					break;
				}
				case 6:
				{
					//AutoLeft6();
					break;
				}
				case 7:
				{
					AutoLeft7();
					break;
				}
				case 8:
				{
					AutoLeft8();
					break;
				}
			
			} //switch (autoprognum)
		
			break;
			
		}//case 1 (2020 Left Modes)

		case 2: //2020 Center Alliance Programs
		{
			switch (autoprognum)
			{
				case 1:
				{
					//AutoCenter1();
					break;
				}
				case 2:
				{
					AutoCenter2();
					break;
				}
				case 3:
				{
					//AutoCenter3();
					break;
				}
				case 4:
				{
					//AutoCenter4();
					break;
				}
				case 5:
				{
					//AutoCenter5();
					break;
				}
				case 6:
				{
					//AutoCenter6();
					break;
				}
				case 7:
				{
					AutoCenter7();
					break;
				}
				case 8:
				{
					AutoCenter8();
					break;
				}
				
			} //switch (autoprognum)			
				
			break;
			
		}//case 2 (2020 Center Modes)

		case 3: //2020 Right Alliance Programs
		{

			switch (autoprognum)
			{
				case 1:
				{
					//AutoRight1();
					break;
				}
				case 2:
				{
					AutoRight2();
					break;
				}
				case 3:
				{
					//AutoRight3();
					break;
				}
				case 4:
				{
					//AutoRight4();
					break;
				}
				case 5:
				{
					//AutoRight5();
					break;
				}
				case 6:
				{
					//AutoRight6();
					break;
				}
				case 7:
				{
					AutoRight7();
					break;
				}
				case 8:
				{
					AutoRight8();
					break;
				}
			
			} //switch (autoprognum)
		
			break;
			
		}//case 3 (2020 Right Modes)
	
	}	

	if (autodriveenable == 1)  //Defaults to on - standard autonomous drive functionality
	{
		
		// Calculate "virtual joystick" x and y values.
		// This is the equivalent of teleop's DoDriveStuff, only using closed loop gyro/encoder control for auton
				
		float xvirt = 0.0;
		float yvirt = 0.0;

		//48 ORIGINAL 2020
		xvirt += -GyroXCtrl->calcPID((int) gyroangle);

		gyrojoy = xvirt;
		
		//48
		yvirt += EncoderYCtrl->calcPID((int) distanceright);
		
		encoderjoy = yvirt;
		
		//printf("xvirt = %f, yvirt = %f\n", xvirt, yvirt);

		// Calculate and output drive values.			
		//Disabled - only if drivetrain shows significant bias to one side
		//LFdrive.speed = LRdrive.speed = DERATEPBOTLEFT * -XM23Lib::limitOutput(yvirt - xvirt); //Left side is inverted
		
		//48
		LFdrive.speed = LRdrive.speed = LMIDdrive.speed = XM23Lib::limitOutput(yvirt - xvirt);
		RFdrive.speed = RRdrive.speed = RMIDdrive.speed = -XM23Lib::limitOutput(yvirt + xvirt);

		//TO DO - COPY THIS CODE TO AUTONOMOUSPERIODIC - AT virtual joystick PID code - enable limelight tracking in auton
		//Limelight Auto-Aim Code
		if ( (shooterstate == ON) && (targetingenabled == 1) )
		{

			//Limelight LED's ON	
			if (targetingenabled == 1) 
			{
				LightsOn();
			}
			
			if (targetdetected == 1) //A vision target has been detected - auto-aim
			{

				if (shiftstate == SHIFTHIGH) 
				{
					GyroXCtrl->setConstants(LIMEGAIN_P, LIMEGAIN_I, LIMEGAIN_D);
				}
				else
				{
					GyroXCtrl->setConstants(LIMEGAIN_P, LIMEGAIN_I, LIMEGAIN_D);
				}
				
				//Limelight Drive
				GyroXCtrl->setDesiredValue(0.0);	

				steercorrect = XM23Lib::limitOutput(GyroXCtrl->calcPID((int) -TX));
			}
			else
			{
				steercorrect = 0.0;
			}
			
			GyroXCtrl->setMaxOutput(DRIVESTRAIGHTMAXCORRECT);

			//48
			//Permit external drive input
			LFdrive.speed =  LFdrive.speed + steercorrect;
			RFdrive.speed =  RFdrive.speed + steercorrect;
			
			//Force a zero-radius turn
			//LFdrive.speed = steercorrect;
			//RFdrive.speed = steercorrect;
		
		}
		else
		{
			steercorrect = 0.0;
			
			GyroXCtrl->setConstants(0.0, 0.0, 0.0);

			//Limelight LED's Off
			LightsOff();

			//DEBUG ONLY 3-13-2021
			//LightsOn();

		}



	driveLF->Set(LFdrive.speed);
	driveLR->Set(LRdrive.speed);
	driveLMID->Set(LMIDdrive.speed);
	
	driveRF->Set(RFdrive.speed);
	driveRR->Set(RRdrive.speed);
	driveRMID->Set(RMIDdrive.speed);

	}	

	DoShiftStuff();
	
	DoSmartStuff();

	DoCANStuff();
	
	//DoArmStuff();

	DoClimbStuff();
	
	DoShooterStuff();

	DoOtherStuff();

	DoCameraStuff();

	DoLEDStuff();
	
	BlinkControl();
	
	//AUTONOMOUS PRINTF DU JOUR
	//printf("lenc = %i, renc = %i, arm = %i, cmd = %i, output = %f, RPM: %i\n", (int) distanceleft, (int) distanceright, armpos, shootercmd, punchspeed, shooterRPM);
	
}


/*******************************************
 ******************************************* 
 *******************************************
 ******   DISABLED CONTINUOUS     ********** 
 *******************************************
 ******************************************* 
 *******************************************/

void XM23::DisabledContinuous(void)
{

	
	
}

/*******************************************
 ******************************************* 
 *******************************************
 ******   TELEOP CONTINUOUS       ********** 
 *******************************************
 ******************************************* 
 *******************************************/

void XM23::TeleopContinuous(void)
{

}


/*******************************************
 ******************************************* 
 *******************************************
 ******   AUTONOMOUS CONTINUOUS   ********** 
 *******************************************
 ******************************************* 
 *******************************************/

void XM23::AutonomousContinuous(void)
{


	
}

void XM23::GetControllerIO(void)
{
	
	/*	DRIVER STATION USB1 CONTROLLER = Logitech Attack 3 Joystick - Left Drive Stick (Example)

		    Joy Y Analog----------------: leftjoyY
		    Joy X Analog----------------: leftjoyX
		    Throttle Analog-------------: speedadjust
		    Button 1 Stick Trigger------: psuck_sw - intake a power cell
		    Button 2 Stick Hat Bottom---: r2d2manual_sw - run R2-D2 manually to generate 60 RPM on color wheel
		    Button 3 Stick Hat Top------: r2d2color_sw - run R2-D2 automatically to correct color (Stage 3)
		    Button 4 Stick Hat Left-----: shiftlow_sw - shifter controls on top of left joystick
		    Button 5 Stick Hat Right----: shifthigh_sw
		    Button 6 Base Left Up-------: slowmode_sw - engage gyro/limelight autocorrect
		    Button 7 Base Left Down-----: blinkinoverride_sw - force the Blinkin to display the selected code
		    Button 8 Base Bottom Left---: joystickselect_sw - select between joystick/gamepad 	 
		    Button 9 Base Bottom Right--: gamepadselect_sw 
		    Button 10 Base Right Down---: autodrive_sw - select automatic drive control (gyro/Limelight-assist)
		    Button 11 Base Right Up-----: manualdrive_sw - select manual drive control (gyro/Limelight-assist disabled)
	*/
	
		leftjoyY			= -drive_joystickL->GetY();
		leftjoyX          	= -drive_joystickL->GetX();
		speedadjust   		= -drive_joystickL->GetRawAxis(2);  //Formerly GetThrottle()
		psuck_sw			= drive_joystickL->GetTrigger();
		r2d2manual_sw		= drive_joystickL->GetTop();
		r2d2color_sw	 	= drive_joystickL->GetRawButton(3);
		shiftlow_sw 		= drive_joystickL->GetRawButton(4);
		shifthigh_sw		= drive_joystickL->GetRawButton(5);
		slowmode_sw		 	= drive_joystickL->GetRawButton(6);
		blinkinoverride_sw  = drive_joystickL->GetRawButton(7);
		joystickselect_sw	= drive_joystickL->GetRawButton(8);
		gamepadselect_sw	= drive_joystickL->GetRawButton(9);
		autodrive_sw 		= drive_joystickL->GetRawButton(10);
		manualdrive_sw		= drive_joystickL->GetRawButton(11);
		
	/*	DRIVER STATION USB2 CONTROLLER = Logitech Attack 3 Joystick - Right Drive Stick (Example)

		    Joy Y Analog----------------: rightjoyY
		    Joy X Analog----------------: rightjoyX
		    Throttle Analog-------------: delayadjust
		    Button 1 Stick Trigger------: pspit_sw - eject power cell from intake
		    Button 2 Stick Hat Bottom---: intakeretract_sw - retract the power cell intake
		    Button 3 Stick Hat Top------: intakedeploy_sw - deploy the power cell intake			
			Button 4 Stick Hat Left-----: ptiltdown_sw - TEMPORARY - manually tilt the arm down
		    Button 5 Stick Hat Right----: ptiltup_sw - TEMPORARY - manually tilt the arm up
			Button 6 Base Left Up-------: climbersafety_sw = press this and both climber buttons to enable climbing/disable other stuff
		    Button 7 Base Left Down-----: 
		    Button 8 Base Bottom Left---: arcade_sw - Select Arcade Mode	 
		    Button 9 Base Bottom Right--: tank_sw - Select Tank Mode
		    Button 10 Base Right Down---: climberetract_sw - retract the climber
		    Button 11 Base Right Up-----: climberextend_sw - extend the climber
	*/
		
		rightjoyY			 = -drive_joystickR->GetY();
		rightjoyX            = -drive_joystickR->GetX();
		delayadjust		 	 = -drive_joystickR->GetRawAxis(2);	 //Formerly GetThrottle()
		pspit_sw			 = drive_joystickR->GetTrigger();
		intakeretract_sw 	 = drive_joystickR->GetTop();
		intakedeploy_sw   	 = drive_joystickR->GetRawButton(3);
		ptiltdown_sw	 	 = drive_joystickR->GetRawButton(4);
		ptiltup_sw	 		 = drive_joystickR->GetRawButton(5);
		climbersafety_sw     = drive_joystickR->GetRawButton(6);
		//spare_sw		     = drive_joystickR->GetRawButton(7);
		arcade_sw 			 = drive_joystickR->GetRawButton(8);
		tank_sw	 			 = drive_joystickR->GetRawButton(9);
		climberetract_sw	 = drive_joystickR->GetRawButton(10);
		climberextend_sw	 = drive_joystickR->GetRawButton(11);

		
	/*	DRIVER STATION USB3 CONTROLLER = Logitech Dual Action Gamepad - Copilot Control (Example)

			Joy 1 X Analog--------------: copadleftjoyX
			Joy 1 Y Analog--------------: copadleftjoyY
			Joy 2 X Analog--------------: copadrightjoyX
			Joy 2 Y Analog--------------: copadrightjoyY
			Button 01 Right Pad Left----: ctrenchfront_sw - select the front of trench shooting smart position
			Button 02 Right Pad Down----: cfarshot_sw - select the far shot shooting smart position 
			Button 03 Right Pad Right---: cballfeedshoot_sw = shoot the power cells based on smart button and other conditions (semi or auto fire)
			Button 04 Right Pad Up------: cspareshot_sw - select the 1/3 field shooting smart position (between wall and trench front shots)
			Button 05 PS2 L1------------: cballfeedload_sw - Load power cells into the conveyor feed (load)
			Button 06 PS2 R1------------: shooteron_sw - Turn the shooter on 
			Button 07 PS2 L2------------: cballfeedeject_sw - Eject power cells from the conveyor feed (unjam)
			Button 08 PS2 R2------------: shooteroff_sw - Turn the shooter off
			Button 09 Select------------: cthroughtrench_sw - Lower the shooter arm to go through trench
			Button 10 Start-------------: cwall_sw - select the adjacent to wall shooting smart position
			Left Stick Push-------------: bigred_sw - reset all smart button functions
			Right Stick Push------------: sbexecute_sw - select chosen smart button function (move right copilot joystick then click it)
			POV Digital Pad-------------: copilotPOV

	*/
			
		copadleftjoyX		= -copilot_pad->GetLeftX();
		copadleftjoyY		= -copilot_pad->GetLeftY();
		copadrightjoyX		= -copilot_pad->GetRightX();
		copadrightjoyY		= -copilot_pad->GetRightY();
		ctrenchfront_sw		= copilot_pad->GetNumberedButton(1);
		cfarshot_sw			= copilot_pad->GetNumberedButton(2);
		cballfeedshoot_sw	= copilot_pad->GetNumberedButton(3);
		cspareshot_sw		= copilot_pad->GetNumberedButton(4);
		cballfeedload_sw	= copilot_pad->GetNumberedButton(5);
		shooteron_sw	    = copilot_pad->GetNumberedButton(6);
		cballfeedeject_sw	= copilot_pad->GetNumberedButton(7);
		shooteroff_sw 		= copilot_pad->GetNumberedButton(8);
		cthroughtrench_sw	= copilot_pad->GetNumberedButton(9);
		cwall_sw			= copilot_pad->GetNumberedButton(10);
		bigred_sw		 	= copilot_pad->GetLeftPush();
		sbexecute_sw		= copilot_pad->GetRightPush();
		//copilotPOV		= copilot_pad->GetPOV();

	/*	DRIVER STATION USB4 CONTROLLER = Logitech Dual Action Gamepad - Pilot Control (Example)

				Joy 1 X Analog--------------: padleftjoyX
				Joy 1 Y Analog--------------: padleftjoyY
				Joy 2 X Analog--------------: padrightjoyX
				Joy 2 Y Analog--------------: padrightjoyY
				Button 01 Right Pad Left----: 
				Button 02 Right Pad Down----:
				Button 03 Right Pad Right---: 
				Button 04 Right Pad Up------:
				Button 05 PS2 L1------------: padshiftlow_sw
				Button 06 PS2 R1------------: padshifthigh_sw
				Button 07 PS2 L2------------: padarmdown_sw
				Button 08 PS2 R2------------: padslowmode_sw - Drive Straight Mode for pilot gamepad
				Button 09 Select------------: 
				Button 10 Start-------------: 
				Left Stick Push-------------: autopos_sw Select autonomous starting position
				Right Stick Push------------: autoprog_sw Select autonomous program for starting position
				POV Digital Pad-------------: pilotPOV
	*/
		
			padleftjoyX			= -pilot_pad->GetLeftX();
			padleftjoyY			= -pilot_pad->GetLeftY();
			padrightjoyX		= -pilot_pad->GetRightX();
			padrightjoyY		= -pilot_pad->GetRightY();		
			//spare_sw		 	= pilot_pad->GetNumberedButton(1);
			//spare_sw 	 		= pilot_pad->GetNumberedButton(2);
			//spare_sw			= pilot_pad->GetNumberedButton(3);
			//spare_sw			= pilot_pad->GetNumberedButton(4);
			padshiftlow_sw 		= pilot_pad->GetNumberedButton(5);
			padshifthigh_sw    	= pilot_pad->GetNumberedButton(6);
			padarmdown_sw		= pilot_pad->GetNumberedButton(7);
			padslowmode_sw		= pilot_pad->GetNumberedButton(8);
			//spare_sw			= pilot_pad->GetNumberedButton(9);
			//spare_sw			= pilot_pad->GetNumberedButton(10);
			//spare_sw			= pilot_pad->GetNumberedButton(9);
			//spare_sw			= pilot_pad->GetNumberedButton(10);
			autopos_sw	 		= pilot_pad->GetLeftPush();
			autoprog_sw			= pilot_pad->GetRightPush();	
			//pilotPOV			= pilot_pad->GetPOV();

		/*
		DRIVER STATION ANALOG INPUTS

		    1:
		    2:
		    3:
		    4:

		DRIVER STATION DIGITAL INPUTS

		    1:
		    2:
		    3:
		    4:
		    5:
		    6:
		    7:
		    8:
			    
		DRIVER STATION DIGITAL OUTPUTS

		    1:
		    2:
		    3:
		    4:
		    5:
		    6:
		    7:
		    8:
			     
	*/	

			//Determine left and right copilot joystick zone states
			
			//	typedef enum{
			//		JOYNEUTRAL, JOYUP, JOYDOWN, JOYLEFT, JOYRIGHT, JOY_UPLEFT, JOY_DOWNLEFT, JOY_DOWNRIGHT, JOY_UPRIGHT
			//	} StickState;
			
			//if ( (copadleftjoyY <= LY_PLUS) && (copadleftjoyY >= LY_MINUS) && (copadleftjoyX <= LX_PLUS) && (copadleftjoyX >= LX_MINUS) ) joyleftstate = JOYNEUTRAL;
			//if ( (copadleftjoyY > LY_PLUS) && (copadleftjoyX <= LX_PLUS) && (copadleftjoyX >= LX_MINUS) ) joyleftstate = JOYUP;
			//if ( (copadleftjoyY < LY_MINUS) && (copadleftjoyX <= LX_PLUS) && (copadleftjoyX >= LX_MINUS) ) joyleftstate = JOYDOWN;
			//if ( (copadleftjoyY <= LY_PLUS) && (copadleftjoyY >= LY_MINUS) && (copadleftjoyX > LX_PLUS)  ) joyleftstate = JOYLEFT;
			//if ( (copadleftjoyY <= LY_PLUS) && (copadleftjoyY >= LY_MINUS) && (copadleftjoyX < LX_MINUS) ) joyleftstate = JOYRIGHT;
			//if ( (copadleftjoyY > LY_PLUS) && (copadleftjoyX > LX_PLUS) ) joyleftstate = JOY_UPLEFT;
			//if ( (copadleftjoyY > LY_PLUS) && (copadleftjoyX < LX_MINUS) ) joyleftstate = JOY_UPRIGHT;
			//if ( (copadleftjoyY < LY_MINUS) && (copadleftjoyX > LX_PLUS) ) joyleftstate = JOY_DOWNLEFT;
			//if ( (copadleftjoyY < LY_MINUS) && (copadleftjoyX < LX_MINUS) ) joyleftstate = JOY_DOWNRIGHT;

			if ( (copadrightjoyY <= RY_PLUS) && (copadrightjoyY >= RY_MINUS) && (copadrightjoyX <= RX_PLUS) && (copadrightjoyX >= RX_MINUS) ) joyrightstate = JOYNEUTRAL;
			if ( (copadrightjoyY > RY_PLUS) && (copadrightjoyX <= RX_PLUS) && (copadrightjoyX >= RX_MINUS) ) joyrightstate = JOYUP;
			if ( (copadrightjoyY < RY_MINUS) && (copadrightjoyX <= RX_PLUS) && (copadrightjoyX >= RX_MINUS) ) joyrightstate = JOYDOWN;
			if ( (copadrightjoyY <= RY_PLUS) && (copadrightjoyY >= RY_MINUS) && (copadrightjoyX > RX_PLUS)  ) joyrightstate = JOYLEFT;
			if ( (copadrightjoyY <= RY_PLUS) && (copadrightjoyY >= RY_MINUS) && (copadrightjoyX < RX_MINUS) ) joyrightstate = JOYRIGHT;
			if ( (copadrightjoyY > RY_PLUS) && (copadrightjoyX > RX_PLUS) ) joyrightstate = JOY_UPLEFT;
			if ( (copadrightjoyY > RY_PLUS) && (copadrightjoyX < RX_MINUS) ) joyrightstate = JOY_UPRIGHT;
			if ( (copadrightjoyY < RY_MINUS) && (copadrightjoyX > RX_PLUS) ) joyrightstate = JOY_DOWNLEFT;
			if ( (copadrightjoyY < RY_MINUS) && (copadrightjoyX < RX_MINUS) ) joyrightstate = JOY_DOWNRIGHT;
			
			//printf (joyrightstate: %f\n", joyrightstate);

} //GetControllerIO

/*******************************************************************
2020 Programs - Left Alliance Starting Position
Left 1 - Full Manual Control
Left 2 - Default - Pick up 2 Balls from Opponent Trench, Shoot 5
Left 3 - 
Left 4 - 
Left 5 - 
Left 6 - 
Left 7 - Drive Forward Only
Left 8 - Do Nothing
*/

void XM23::AutoLeft1(void)
{
	//printf("Left 1 - Full Manual Control)\n");

	autodriveenable = 0;	//Full Manual Control

}

void XM23::AutoLeft2(void)
{
	//printf("Left 2 - Pick up 2 Balls from Opponent Trench, Shoot 5\n");

}

void XM23::AutoLeft3(void)
{
	//printf("Left 3 - Spare\n");

}

void XM23::AutoLeft4(void)
{
	//printf("Left 4 - Spare\n");

}

void XM23::AutoLeft5(void)
{
	//printf("Left 5 - Spare\n");

}

void XM23::AutoLeft6(void)
{
	//printf("Left 6 - Spare\n");

}

void XM23::AutoLeft7(void)
{
	//printf("Left 7 - Drive Backward Only\n");

	#define L7_STEER		0.50	//0.50 - Max steer correction
	#define L7_SPEED1	 	0.70	//0.70 - Medium Speed
	#define L7_SPEED2		0.50	//0.50 - Slow Speed
	#define L7_SPEED3		0.90	//0.90 - Fast Speed
	#define L7_STRAIGHTDIR  0.0		//0.0  - Straightline travel angle command

	#define L7_SCOREDELAY	1.00	//1.00 - Pause Time for setting systems to smart home mode
	#define L7_REV1			-45		//-30  - Drive Backward

	switch (autonStep)
	{

		case 1: //Set home smartbutton
		{
			//printf("Step 1!\n");

			//Put the system in close (initiation line) shot position
			autospareshot_sw = 1;

			if (autotimer->Get() > L7_SCOREDELAY)
			{

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 2;

			}

			break;
		}
		case 2: //Drive backward
		{

			//printf("Step 2!\n");

			//Move forward
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(L7_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		
			GyroXCtrl->setMinDoneCycles(2);     
			GyroXCtrl->setMaxOutput(L7_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(L7_REV1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(L7_SPEED1);

			if (EncoderYCtrl->isDone())
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 3;

			}

			break;
		}
		case 3: //All done!
		{

			//printf("Step 3!\n");

			BigRedBash();

			ResetAutoVars();

			break;

		}

	} //End Switch

}

void XM23::AutoLeft8(void)
{
	//printf("Left 8 - Do Nothing\n");

}


/*******************************************************************
2020 Programs - Center Alliance Starting Position
Center 1 - Full Manual Control
Center 2 - Default - Shoot 3 Balls, Move off Initiation Line
Center 3 - 
Center 4 - 2021 AUTONAV CHALLENGE - LOW GEAR BOUNCE PATH (BALLS)
Center 5 - 2021 AUTONAV CHALLENGE - LOW GEAR SLALOM PATH (FISH)
Center 6 - 
Center 7 - Drive Forward Only
Center 8 - Do Nothing
*/


void XM23::AutoCenter1(void)
{
	//printf("Center 1 - Full Manual Control)\n");

	autodriveenable = 0;	//Full Manual Control

}

void XM23::AutoCenter2(void)
{
	//printf("Center 2 - Shoot 3 Balls, Move off Initiation Line)\n");

	#define C2_STEER		0.50	//0.50 - Max steer correction 
	#define C2_SPEED1	 	0.70	//0.70 - Medium Speed
	#define C2_SPEED2		0.50	//0.50 - Medium Speed
	#define C2_SPEED3		0.90	//0.90 - Medium Speed
	#define C2_STRAIGHTDIR  0.0		//0.0  - Straightline travel angle command

	#define C2_READYDWELL	2.5		//Watchdog dwell time waiting for ok to shoot signal
	#define C2_SHOTDELAY	0.1		//Short pause between oktoshoot signal and feeding ball
	#define	C2_SHOTDWELL	2.0		//5.0 OK - Feed dwell time for all 3 balls (assuming no major regen needed)
	#define C2_DEPLOYDWELL	1.0		//1.0 OK - Dwell time to run intake to ensure flip-out deployment

	#define C2_REV1			-60		//-60  - Drive Backward from initiation line	
	#define C2_TURNDIR1		-30		//-90  - Arc turn to point toward trench
	
	switch (autonStep)
	{

		case 1: //Set home shot position mode and enable limelight tracking
		{
			//printf("Step 1!\n");

			//Put the system in close (initiation line) shot position
			autospareshot_sw = 1;

			if (delaytimer->Get() < delayset) 	//Wait until the timer times past the preset delay
			{

				//Waiting...
				
			}
			else
			{

				delaytimer->Stop();

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 2;
			}

			break;
		}
		case 2: //Wait for shooter to be ready (or watchdog timeout) 
		{				
			//printf("Step 2!\n");	

			autoshooteron_sw = 1;	//Turn on the shooter (which should also begin Limelight tracking)
			autoshooteroff_sw = 0; 

			if ( (oktoshoot == 1) || (autotimer->Get() > C2_READYDWELL) )
			{	
				autotimer->Reset();

				autonStep = 3;
			}
		
			break;
		}		
		case 3: //Wait for a short moment after shooter is ready before firing all balls (2012 legacy - minimize duration)
		{	
			//printf("Step 3!\n");			
			
			if (autotimer->Get() > C2_SHOTDELAY) 
			{
				autoballfeedshoot_sw = 1;
				
				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 4;
			}
		
			break;
		}		
		case 4: //Stop shooting after the shot dwell timeout
		{
			//printf("Step 4!\n");

			if (autotimer->Get() > C2_SHOTDWELL)
			{
				autoballfeedshoot_sw = 0;		//Stop shooting balls

				autoshooteroff_sw = 1;			//Turn off the shooter
				autoshooteron_sw = 0; 
				
				autotimer->Reset();
				
				RightEncoder->Reset();

				gyro->Reset();
				
				autonStep = 5;
			}
						
			break;
		}		
		case 5: //Arc Turn Backwards toward the trench
		{
			//printf("Step 5!\n");

			//Arc turn backwards left toward the trench
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C2_TURNDIR1);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C2_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C2_REV1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C2_SPEED1);

			if (EncoderYCtrl->isDone())
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 6;

			}

			break;
		}
		case 6: //Deploy and run intake for a short bit to ensure deployment
		{

			//printf("Step 6!\n");

			autointakedeploy_sw = 1;

			autosuck_sw = 1;

			if (autotimer->Get() > C2_DEPLOYDWELL)
			{
				autosuck_sw = 0;		//Turn off the intake
				
				BigRedBash();

				ResetAutoVars();

			}

			break;

		}

	} //End Switch

}

void XM23::AutoCenter3(void)
{
	//printf("Center 3 - Spare\n");

}

void XM23::AutoCenter4(void)
{
	//printf("Center 4 - LOW GEAR AutoNav Bounce Path (BALLS)\n");

	#define C4_STEER		0.50	//0.50 - Max steer correction 
	#define C4_SPEED1	 	0.70	//0.70 - Medium Speed
	#define C4_SPEED2		0.50	//0.50 - Slow Speed
	#define C4_SPEED3		0.90	//0.90 - Fast Speed
	#define C4_STRAIGHTDIR  0.0		//0.0  - Straightline travel angle command

	#define C4_READYDWELL	 0.5	//Delay time waiting for arm to lower safely
	
	#define C4_FWD1			 18		// 18 - Drive forward before 1st turn

	#define C4_FWD2			 90		// 90 - Drive toward 1st ball
	#define C4_TURNDIR1		-110	//-110 - Turn Left toward 1st ball

	#define C4_REV1			-103	//-103 - Drive backward toward 2nd ball arc turn start
	
	#define C4_REV2			-70		//-70 - Drive backward toward 2nd ball
	#define C4_TURNDIR2		-160 	//-160 - Turn Left toward 2nd ball

	#define C4_REV3			-80		//-80 - Drive backward to contact 2nd ball

	#define C4_FWD3			 100	// 100 - Drive forward toward 3rd ball arc turn start - SMALL LEFT TURN
	#define C4_TURNDIR3		-10		//-10 - Turn Left toward 2nd ball

	#define C4_FWD4			 100 	// 100 - Drive forward toward 3rd ball - step 1
	#define C4_TURNDIR4		-90	    //-90 - Turn Left toward 3rd ball

	//NEW STEP ADDED
	#define C4_REV5			 80		// 80 - Drive forward toward 3rdd ball - step 2
	#define C4_TURNDIR6		-75 	//-75 - Turn Left toward 2nd ball
	//**************

	#define C4_FWD5			 72		// 72 - Drive forward to contact the 3rd ball (possible left slight arc turn)

	#define C4_REV4			-62		// -62 - Drive backward to cross end zone
	#define C4_TURNDIR5		-18 	// -18 - Turn Left toward End Zone


	switch (autonStep)
	{

		case 1: //Set trenchrun position mode and shift to low gear
		{
			//printf("Step 1!\n");

			autothroughtrench_sw = 1;			//Lower the arm and retract the hood

			shiftstate = SHIFTLOW;

			autotimer->Reset();

			RightEncoder->Reset();

			gyro->Reset();

			autonStep = 2;

			break;
		}
		case 2: //Wait for arm to be ready (or watchdog timeout) 
		{				
			//printf("Step 2!\n");	

			if (autotimer->Get() > C4_READYDWELL)
			{	
				autotimer->Reset();

				autonStep = 3;
			}
		
			break;
		}		
		case 3: //Drive straight forward before left turn toward 1st ball)
		{	
			//printf("Step 3!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_FWD1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 4;

			}

			break;

		}		
		case 4: //Execute left turn toward first ball and contact it
		{
			//printf("Step 4!\n");

			//Arc turn forward left toward the first ball
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_TURNDIR1);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_FWD2);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 5;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}		
		case 5: //Drive Straight Backwards to prepare for arc turn toward 2nd ball 
		{
			//printf("Step 5!\n");

			//Drive straight
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_REV1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 6;

			}

			break;

		}
		case 6: //Make a reverse arc turn and align straight on with the 2nd ball
		{

			//printf("Step 6!\n");

			//Arc turn reverse left toward the second ball
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_TURNDIR2);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_REV2);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 7;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		case 7: //Drive backward to contact the 2nd ball
		{

			//printf("Step 7!\n");

			//Drive straight backward to contact the 2nd ball
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_REV3);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 8;

			}

			break;

		}
		case 8: //Drive slight left turn forward toward the start of the arc turn to the 3rd ball 
		{				
			//printf("Step 8!\n");	

			//Drive straight
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_TURNDIR3);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_FWD3);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 9;

			}

			break;

		}		
		case 9: //Make the arc turn to align with the 3rd ball
		{	
			//printf("Step 9!\n");			
				
			//Arc turn forward left toward the third ball
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_TURNDIR4);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_FWD4);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				//JUMP TO NEW APPENDED STEP BELOW
				autonStep = 13;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		//NEW STEP - case number out of sequence intentionally
		case 13: //Make the arc turn to align with the 3rd ball
		{	
			//printf("Step 13!\n");			
				
			//Arc turn forward left toward the third ball
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_TURNDIR6);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_FWD5);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 10;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 10: //Drive straight forward to contact the 3rd ball
		{
			//printf("Step 10!\n");

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_FWD5);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 11;

			}

			break;

		}
		case 11: //Make a reverse arc turn to cross the end zone
		{

			//printf("Step 11!\n");

			//Arc turn reverse left to enter the End Zone
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C4_TURNDIR5);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C4_REV4);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C4_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 12;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		case 12: //All done!
		{
			//printf("Step 12!\n");

			BigRedBash();

			ResetAutoVars();

			break;

		}	

	} //End Switch


}

void XM23::AutoCenter5(void)
{
	//printf("Center 5 - LOW GEAR AutoNav Slalom Path (FISH)\n");

	#define C5_STEER		0.60	//0.60 - Max steer correction 
	#define C5_SPEED1	 	0.80	//0.80 - Medium Speed
	#define C5_SPEED2		0.60	//0.60 - Slow Speed
	#define C5_SPEED3		1.00	//1.00 - Fast Speed
	#define C5_STRAIGHTDIR  0.0		//0.0  - Straightline travel angle command

	#define C5_READYDWELL	 0.5	//Delay time waiting for arm to lower safely
	
	#define C5_FWD1			 18		// 18 - Drive forward before 1st turn

	#define C5_FWD2			 75		// 75 - Drive forward to execute 1st left slalom
	#define C5_TURNDIR1		-60	    //-60 - Turn Left for 1st left slalom

	#define C5_FWD3			 43		// 43 - Drive forward to execute 1st right slalom
	#define C5_TURNDIR2		 60 	// 60 - Turn Right for 1st right slalom

	#define C5_FWD4			 90	    // 90 - Drive forward toward 2nd right slalom

	#define C5_FWD5			 55		// 55 - Drive forward to execute 2nd right slalom
	#define C5_TURNDIR3		 60 	// 60 - Turn Right for 2nd right slalom

	#define C5_FWD6			 55		// 43 - Drive forward to execute 2nd left slalom
	#define C5_TURNDIR4		-60	    //-60 - Turn Left for 2nd left slalom

	#define C5_FWD7			 132	// 150 - Drive forward to execute 180 degree turn around far pylon
	#define C5_TURNDIR5		-172    //-180 - Turn Left for 180 degree turn around far pylon

	#define C5_FWD8			 75		// 75 - Drive forward to execute 3rd left slalom
	#define C5_TURNDIR6		-60	    //-60 - Turn Left for 3rd left slalom

	#define C5_FWD9			 48		// 48 - Drive forward to execute 3rd right slalom
	#define C5_TURNDIR7		 63 	// 60 - Turn Right for 3rd right slalom

	#define C5_FWD10		 80	    // 85 - Drive forward toward 4th right slalom

	#define C5_FWD11		 43		// 43 - Drive forward to execute 4th right slalom
	#define C5_TURNDIR8		 60 	// 50 - Turn Right for 4th right slalom

	#define C5_FWD12		 48	    // 12 - Drive forward to cross End Zone line

	switch (autonStep)
	{

		case 1: //Set trenchrun position mode and shift to high gear
		{
			//printf("Step 1!\n");

			autothroughtrench_sw = 1;			//Lower the arm and retract the hood

			shiftstate = SHIFTLOW;

			autotimer->Reset();

			RightEncoder->Reset();

			gyro->Reset();

			autonStep = 2;

			break;
		}
		case 2: //Wait for arm to be ready (or watchdog timeout) 
		{				
			//printf("Step 2!\n");	

			if (autotimer->Get() > C5_READYDWELL)
			{	
				autotimer->Reset();

				autonStep = 3;
			}
		
			break;
		}		
		case 3: //Drive straight forward before 1st slalom left turn
		{	
			//printf("Step 3!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 4;

			}

			break;

		}		
		case 4: //Execute 1st left slalom turn
		{
			//printf("Step 4!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_TURNDIR1);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD2);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 5;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}		
		case 5: //Execute 1st right slalom turn
		{
			//printf("Step 5!\n");

			//Arc turn forward right
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_TURNDIR2);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD3);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 6;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 6: //Drive straight forward toward 2nd slalom right turn
		{	
			//printf("Step 6!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD4);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 7;

			}

			break;

		}
		case 7: //Execute 2nd right slalom turn
		{
			//printf("Step 7!\n");

			//Arc turn forward right
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_TURNDIR3);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD5);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 8;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 8: //Execute 2nd left slalom turn
		{
			//printf("Step 8!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_TURNDIR4);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD6);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 9;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}		
		case 9: //Execute 180 left turn around far pylon 
		{
			//printf("Step 9!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_TURNDIR5);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD7);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 10;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 10: //Execute 3rd left slalom turn
		{
			//printf("Step 10!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_TURNDIR6);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD8);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 11;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}		
		case 11: //Execute 3rd right slalom turn
		{
			//printf("Step 11!\n");

			//Arc turn forward right
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_TURNDIR7);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD9);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 12;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		case 12: //Drive straight forward toward 4th slalom right turn
		{	
			//printf("Step 12!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD10);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 13;

			}

			break;

		}
		case 13: //Execute 4th right slalom turn
		{
			//printf("Step 13!\n");

			//Arc turn forward right
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_TURNDIR8);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD11);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 14;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 14: //Drive straight forward to cross End Zone line
		{	
			//printf("Step 14!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C5_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(C5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C5_FWD12);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C5_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 15;

			}

			break;

		}
		case 15: //All done!
		{
			//printf("Step 15!\n");

			BigRedBash();

			ResetAutoVars();

			break;

		}	

	} //End Switch


}

void XM23::AutoCenter6(void)
{
	//printf("Center 6 - Spare\n");

}

void XM23::AutoCenter7(void)
{
	//printf("Center 7 - Drive Backward Only\n");

	#define C7_STEER		0.50	//0.50 - Max steer correction
	#define C7_SPEED1	 	0.70	//0.70 - Medium Speed
	#define C7_SPEED2		0.50	//0.50 - Slow Speed
	#define C7_SPEED3		0.90	//0.90 - Fast Speed
	#define C7_STRAIGHTDIR  0.0		//0.0  - Straightline travel angle command

	#define C7_SCOREDELAY	1.00	//1.00 - Pause Time for setting systems to smart home mode
	#define C7_REV1			-45		//-30  - Drive Backward

	switch (autonStep)
	{

		case 1: //Set home smartbutton
		{
			//printf("Step 1!\n");

			//Put the system in close (initiation line) shot position
			autospareshot_sw = 1;

			if (autotimer->Get() > C7_SCOREDELAY)
			{

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 2;

			}

			break;
		}
		case 2: //Drive backward
		{
			//printf("Step 2!\n");

			//Move forward
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(C7_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		
			GyroXCtrl->setMinDoneCycles(2);     
			GyroXCtrl->setMaxOutput(C7_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(C7_REV1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(C7_SPEED1);

			if (EncoderYCtrl->isDone())
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 3;

			}

			break;
		}
		case 3: //All done!
		{
			//printf("Step 3!\n");

			BigRedBash();

			ResetAutoVars();

			break;

		}

	} //End Switch

}

void XM23::AutoCenter8(void)
{
	//printf("Center 8 - Do Nothing\n");

}



/*******************************************************************
2020 Programs - Right Alliance Starting Position
Right 1 - Full Manual Control
Right 2 - Default - Shoot 3 Balls, Gather 3 from Trench, Shoot 3
Right 3 - 
Right 4 - 2021 AUTONAV CHALLENGE - HIGH GEAR BOUNCE PATH (BALLS)
Right 5 - 2021 AUTONAV CHALLENGE - HIGH GEAR SLALOM PATH (FISH)
Right 6 - 2021 AUTONAV CHALLENGE - HIGH GEAR BARREL RACING PATH
Right 7 - Drive Forward Only
Right 8 - Do Nothing
*/

void XM23::AutoRight1(void)
{
	//printf("Right 1 - Full Manual Control\n");

	autodriveenable = 0;	//Full Manual Control

}

void XM23::AutoRight2(void)
{
	//printf("Right 2 - Shoot 3 Balls, Gather 3 from Trench, Shoot 3\n");

	#define R2_STEER		0.50	//0.50 - Max steer correction 
	#define R2_SPEED1	 	0.70	//0.70 - Slow Speed
	#define R2_SPEED2		0.90	//0.90 - Medium Speed
	#define R2_SPEED3		1.00	//0.90 - Full Speed
	#define R2_STRAIGHTDIR  0.0		//0.0  - Straightline travel angle command

	#define R2_READYDWELL	2.0		//2.0 OK - Watchdog dwell time waiting for ok to shoot signal
	#define R2_SHOTDELAY	0.1		//0.1 OK - Short pause between oktoshoot signal and feeding ball
	#define	R2_SHOTDWELL	1.7		//1.7 OK - Feed dwell time for 3 balls (assuming no major regen needed)

	#define R2_PULSEDELAY   0.01    //0.01 OK - Delay during forward move to 2nd shot position before running ball feed backwards
	#define R2_PULSEDWELL	1.0     //0.7  OK - Dwell time to run the ball feed backwards to prevent power cells touching shooter motors
	#define R2_STOPDWELL	1.5     //1.2  OK - Delay for shooter to stop reversing before turning shooter on

	#define R2_READYDWELL2	0.5		//0.5 OK - Delay time before shooting 3 trench balls - act fast!

	#define R2_REV1			-60		//-60 - Drive Backward from initiation line
	#define R2_TURNDIR1		-27		//-27 - Arc turn to point toward trench
	
	#define R2_REV2			-71		//-71 - Drive backward to collect 1st trench ball
	
	#define R2_TURNDIR2		27		// 27 - Turn right in place to align with last 2 trench balls

	#define R2_REV3			-85		//-85 - Drive backwards to collect last 2 trench balls
	
	#define R2_FWD1			170		//170 - Drive forward to final shooting location
	#define R2_TURNDIR3		-10		//-10 - Arc turn Left to align with goal
	
	switch (autonStep)
	{

		case 1: //Set home shot position mode and enable limelight tracking
		{
			//printf("Step 1!\n");

			//Put the system in close (initiation line) shot position
			autospareshot_sw = 1;

			//if (delaytimer->Get() < delayset) 	//Wait until the timer times past the preset delay
			//{

				//Waiting...
				
			//}
			//else
			//{

			//delaytimer->Stop();

			autotimer->Reset();

			RightEncoder->Reset();

			gyro->Reset();

			autonStep = 2;

			//}

			break;
		}
		case 2: //Wait for shooter to be ready (or watchdog timeout) 
		{				
			//printf("Step 2!\n");	

			autoshooteron_sw = 1;	//Turn on the shooter (which should also begin Limelight tracking)
			autoshooteroff_sw = 0; 

			if ( (oktoshoot == 1) || (autotimer->Get() > R2_READYDWELL) )
			{	
				autotimer->Reset();

				autonStep = 3;
			}
		
			break;
		}		
		case 3: //Wait for a short moment after shooter is ready before firing all balls (2012 legacy - minimize duration)
		{	
			//printf("Step 3!\n");			
			
			if (autotimer->Get() > R2_SHOTDELAY) 
			{
				autoballfeedshoot_sw = 1;
				
				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 4;
			}
		
			break;
		}		
		case 4: //Stop shooting after the shot dwell timeout
		{
			//printf("Step 4!\n");

			if (autotimer->Get() > R2_SHOTDWELL)
			{
				autoballfeedshoot_sw = 0;		//Stop shooting balls

				autoshooteroff_sw = 1;			//Turn off the shooter
				autoshooteron_sw = 0; 

				autotimer->Reset();
				
				RightEncoder->Reset();

				gyro->Reset();
				
				autonStep = 5;
			}
						
			break;
		}		
		case 5: //Arc Turn Backwards toward the trench
		{
			//printf("Step 5!\n");

			//Arc turn backwards left toward the trench
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(R2_TURNDIR1);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R2_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R2_REV1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R2_SPEED3);

			if (EncoderYCtrl->isDone())
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 6;

			}

			break;
		}
		case 6: //Deploy intake, enable intake and ball feed, drive straight backwards to collect the 1st trench ball
		{

			//printf("Step 6!\n");

			//Deploy the intake
			autointakedeploy_sw = 1;

			//Turn on the intake
			autosuck_sw = 1;

			//Turn on the ball feed to collect
			//autoballfeedshoot_sw = 0;
			autoballfeedload_sw = 1;

			//Drive straight backwards to collect the first trench ball
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(R2_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R2_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R2_REV2);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R2_SPEED3);

			if (EncoderYCtrl->isDone())
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 7;

			}

			break;

		}
		case 7: //Turn right to point toward final 2 trench balls
		{

			//printf("Step 7!\n");

			//Turn on the intake
			//autosuck_sw = 1;

			//Turn on the ball feed to collect
			//autoballfeedshoot_sw = 0;
			//autoballfeedload_sw = 1;

			//Turn right to point toward final 2 trench balls
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(R2_TURNDIR2);
			GyroXCtrl->setErrorEpsilon(2);
			GyroXCtrl->setMinDoneCycles(2);
			GyroXCtrl->setMaxOutput(R2_STEER);

			if (GyroXCtrl->isDone())
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 8;

			}

			break;

		}
		case 8: //Drive straight backwards to collect the last 2 trench balls, then turn off ball feed and intake
		{

			//printf("Step 8!\n");

			//Turn on the intake
			//autosuck_sw = 1;

			//Turn on the ball feed to collect
			//autoballfeedshoot_sw = 0;
			//autoballfeedload_sw = 1;

			//Drive straight backwards to collect the last 2 trench balls
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(R2_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R2_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R2_REV3);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R2_SPEED2);

			if (EncoderYCtrl->isDone())
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				//autosuck_sw = 0;

				autoballfeedload_sw = 0;

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 9;

			}

			break;

		}		
		case 9: //Forward slight arc turn toward shooting location, pulsing the ball feed backwards to ensure a gap between power cells and shooter wheel, then turn shooter on
		{

			//printf("Step 9!\n");

			//Arc turn forward left toward the goal target
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(R2_TURNDIR3);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R2_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R2_FWD1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R2_SPEED3);

			if (autotimer->Get() <= R2_PULSEDELAY)	//Turn on the ball feed in reverse
			{
				autoballfeedeject_sw = 1;
			}
			else if (autotimer->Get() <= R2_PULSEDWELL)  //Turn off the ball feed if reverse pulse timer is complete
			{
				autoballfeedeject_sw = 0;
			}
			else if (autotimer->Get() <= R2_STOPDWELL)   //Turn on the shooter
			{
				autoshooteron_sw = 1;
				autoshooteroff_sw = 0;   
			}

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autosuck_sw = 0;  //Delayed turning the intake off to help keep balls inside bot during forward movement

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 10;

			}

			break;

		}
		case 10: //Wait for shooter to be ready (or watchdog timeout) 
		{				
			//printf("Step 10!\n");	

			if ( (oktoshoot == 1) || (autotimer->Get() > R2_READYDWELL2) )
			{	
				autotimer->Reset();

				autonStep = 11;
			}
		
			break;

		}		
		case 11: //Wait for a short moment after shooter is ready before firing all balls (2012 legacy - minimize duration)
		{	
			//printf("Step 11!\n");			
			
			if (autotimer->Get() > R2_SHOTDELAY) 
			{
				autoballfeedshoot_sw = 1;
				
				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 12;
			}
		
			break;

		}	
		case 12: //Stop shooting after the shot dwell timeout
		{
			//printf("Step 12!\n");

			if (autotimer->Get() > R2_SHOTDWELL)
			{
				autoballfeedshoot_sw = 0;		//Stop shooting balls

				autoshooteroff_sw = 1;			//Turn off the shooter
				autoshooteron_sw = 0; 

				autotimer->Reset();
				
				RightEncoder->Reset();

				gyro->Reset();
				
				autonStep = 13;
			}
						
			break;

		}
		case 13: //All done!
		{
			//printf("Step 13!\n");

			BigRedBash();

			ResetAutoVars();

			break;

		}	

	} //End Switch

}

void XM23::AutoRight3(void)
{
	//printf("Right 3 - Spare\n");

}

void XM23::AutoRight4(void)
{
	//printf("Right 4 - HIGH GEAR AutoNav Bounce Path (BALLS)\n");

	#define R4_STEER		0.20	//0.23 - Max steer correction 
	#define R4_SPEED1	 	0.40	//0.40 - Medium Speed
	#define R4_SPEED2		0.30	//0.30 - Slow Speed
	#define R4_SPEED3		0.50	//0.50 - Fast Speed
	#define R4_STRAIGHTDIR  0.0		//0.0  - Straightline travel angle command

	#define R4_READYDWELL	 0.1	//Delay time waiting for arm to lower safely
	
	#define R4_FWD1			 18		// 18 - Drive forward before 1st turn

	#define R4_FWD2			 90		// 90 - Drive toward 1st ball
	#define R4_TURNDIR1		-112	//-112 - Turn Left toward 1st ball

	#define R4_REV1			-103	//-103 - Drive backward toward 2nd ball arc turn start
	
	#define R4_REV2			-80	    //-70 - Drive backward toward 2nd ball
	#define R4_TURNDIR2		-160 	//-160 - Turn Left toward 2nd ball

	#define R4_REV3			-80		//-80 - Drive backward to contact 2nd ball

	#define R4_FWD3			 90 	// 90 - Drive forward toward 3rd ball arc turn start - SMALL LEFT TURN
	#define R4_TURNDIR3		-5		//-5 - Turn Left toward 2nd ball

	#define R4_FWD4			 100 	// 100 - Drive forward toward 3rd ball - step 1
	#define R4_TURNDIR4		-90	    //-90 - Turn Left toward 3rd ball

	//NEW STEP ADDED
	#define R4_FWD5			 80		// 80 - Drive forward toward 3rd ball - step 2
	#define R4_TURNDIR6		-75 	//-75 - Turn Left toward 2nd ball
	//**************

	#define R4_FWD6			 65		// 72 - Drive forward to contact the 3rd ball (possible left slight arc turn)

	#define R4_REV4			-40		// -40 - Drive backward to cross end zone
	#define R4_TURNDIR5		-18 	// -18 - Turn Left toward End Zone


	switch (autonStep)
	{

		case 1: //Set trenchrun position mode and shift to low gear
		{
			//printf("Step 1!\n");

			autothroughtrench_sw = 1;			//Lower the arm and retract the hood

			shiftstate = SHIFTHIGH;

			autotimer->Reset();

			RightEncoder->Reset();

			gyro->Reset();

			autonStep = 2;

			break;
		}
		case 2: //Wait for arm to be ready (or watchdog timeout) 
		{				
			//printf("Step 2!\n");	

			if (autotimer->Get() > R4_READYDWELL)
			{	
				autotimer->Reset();

				autonStep = 3;
			}
		
			break;
		}		
		case 3: //Drive straight forward before left turn toward 1st ball)
		{	
			//printf("Step 3!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_FWD1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED1);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 4;

			}

			break;

		}		
		case 4: //Execute left turn toward first ball and contact it
		{
			//printf("Step 4!\n");

			//Arc turn forward left toward the first ball
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_TURNDIR1);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_FWD2);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED2);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 5;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}		
		case 5: //Drive Straight Backwards to prepare for arc turn toward 2nd ball 
		{
			//printf("Step 5!\n");

			//Drive straight
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_REV1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 6;

			}

			break;

		}
		case 6: //Make a reverse arc turn and align straight on with the 2nd ball
		{

			//printf("Step 6!\n");

			//Arc turn reverse left toward the second ball
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_TURNDIR2);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_REV2);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED3);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 7;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		case 7: //Drive backward to contact the 2nd ball
		{

			//printf("Step 7!\n");

			//Drive straight backward to contact the 2nd ball
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_REV3);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 8;

			}

			break;

		}
		case 8: //Drive slight left turn forward toward the start of the arc turn to the 3rd ball 
		{				
			//printf("Step 8!\n");	

			//Drive straight
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_TURNDIR3);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_FWD3);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 9;

			}

			break;

		}		
		case 9: //Make the arc turn to align with the 3rd ball
		{	
			//printf("Step 9!\n");			
				
			//Arc turn forward left toward the third ball
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_TURNDIR4);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_FWD4);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED1);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				//JUMP TO NEW APPENDED STEP BELOW
				autonStep = 13;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		//NEW STEP - case number out of sequence intentionally
		case 13: //Make the arc turn to align with the 3rd ball
		{	
			//printf("Step 13!\n");			
				
			//Arc turn forward left toward the third ball
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_TURNDIR6);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_FWD5);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED1);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 10;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 10: //Drive straight forward to contact the 3rd ball
		{
			//printf("Step 10!\n");

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_FWD6);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED3);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 11;

			}

			break;

		}
		case 11: //Make a reverse arc turn to cross the end zone
		{

			//printf("Step 11!\n");

			//Arc turn reverse left to enter the End Zone
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R4_TURNDIR5);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R4_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R4_REV4);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R4_SPEED1);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 12;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		case 12: //All done!
		{
			//printf("Step 12!\n");

			BigRedBash();

			ResetAutoVars();

			break;

		}	

	} //End Switch

}

void XM23::AutoRight5(void)
{
	//printf("Right 5 - HIGH GEAR AutoNav Slalom Path (FISH)\n");

	#define R5_STEER		0.25	//0.30 - Max steer correction 

	#define R5_SPEED30		0.30    //0.30
	#define R5_SPEED40	 	0.40	//0.40 
	#define R5_SPEED50		0.50	//0.50 
	#define R5_SPEED60		0.60	//0.60
	#define R5_SPEED70		0.70	//0.70
	#define R5_SPEED80		0.80	//0.80
	#define R5_SPEED90		0.90	//0.90
	#define R5_SPEED100     1.00	//1.00

	#define R5_STRAIGHTDIR   0.0	//0.0  - Straightline travel angle command

	#define R5_READYDWELL	 0.05	//Delay time waiting for arm to lower safely
	
	#define R5_FWD1			 12		// 12 - Drive forward before 1st turn

	#define R5_FWD2			 70		// 70 - Drive forward to execute 1st left slalom
	#define R5_TURNDIR1		-60	    //-60 - Turn Left for 1st left slalom

	#define R5_FWD3			 43		// 43 - Drive forward to execute 1st right slalom
	#define R5_TURNDIR2		 65 	// 65 - Turn Right for 1st right slalom

	#define R5_FWD4			 80	    // 80 - Drive forward toward 2nd right slalom

	#define R5_FWD5			 60		// 60 - Drive forward to execute 2nd right slalom
	#define R5_TURNDIR3		 60 	// 60 - Turn Right for 2nd right slalom

	#define R5_FWD6			 60		// 60 - Drive forward to execute 2nd left slalom
	#define R5_TURNDIR4		-60	    //-60 - Turn Left for 2nd left slalom

	#define R5_FWD7			 145    // 145 - Drive forward to execute 180 degree turn around far pylon
	#define R5_TURNDIR5		-180    //-180 - Turn Left for 180 degree turn around far pylon

	#define R5_FWD8			 70		// 70 - Drive forward to execute 3rd left slalom
	#define R5_TURNDIR6		-60	    //-60 - Turn Left for 3rd left slalom

	#define R5_FWD9			 48		// 48 - Drive forward to execute 3rd right slalom
	#define R5_TURNDIR7		 65 	// 65 - Turn Right for 3rd right slalom

	#define R5_FWD10		 70	    // 70 - Drive forward toward 4th right slalom

	#define R5_FWD11		 40		// 40 - Drive forward to execute 4th right slalom
	#define R5_TURNDIR8		 65 	// 65 - Turn Right for 4th right slalom

	#define R5_FWD12		 48	    // 48 - Drive forward to cross End Zone line

	switch (autonStep)
	{

		case 1: //Set trenchrun position mode and shift to high gear
		{
			//printf("Step 1!\n");

			autothroughtrench_sw = 1;			//Lower the arm and retract the hood

			shiftstate = SHIFTHIGH;

			autotimer->Reset();

			RightEncoder->Reset();

			gyro->Reset();

			autonStep = 2;

			break;
		}
		case 2: //Wait for arm to be ready (or watchdog timeout) 
		{				
			//printf("Step 2!\n");	

			if (autotimer->Get() > R5_READYDWELL)
			{	
				autotimer->Reset();

				autonStep = 3;
			}
		
			break;
		}		
		case 3: //Drive straight forward before 1st slalom left turn
		{	
			//printf("Step 3!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED40);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 4;

			}

			break;

		}		
		case 4: //Execute 1st left slalom turn
		{
			//printf("Step 4!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_TURNDIR1);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD2);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED40);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 5;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}		
		case 5: //Execute 1st right slalom turn
		{
			//printf("Step 5!\n");

			//Arc turn forward right
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_TURNDIR2);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD3);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED40);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 6;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 6: //Drive straight forward toward 2nd slalom right turn
		{	
			//printf("Step 6!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD4);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED50);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 7;

			}

			break;

		}
		case 7: //Execute 2nd right slalom turn
		{
			//printf("Step 7!\n");

			//Arc turn forward right
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_TURNDIR3);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD5);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED40);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 8;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 8: //Execute 2nd left slalom turn
		{
			//printf("Step 8!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_TURNDIR4);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD6);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED40);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 9;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}		
		case 9: //Execute 180 left turn around far pylon 
		{
			//printf("Step 9!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_TURNDIR5);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(0.22);		//OVERRIDE R5_STEER JUST FOR THIS TURN

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD7);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED50);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 10;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 10: //Execute 3rd left slalom turn
		{
			//printf("Step 10!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_TURNDIR6);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD8);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED40);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 11;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}		
		case 11: //Execute 3rd right slalom turn
		{
			//printf("Step 11!\n");

			//Arc turn forward right
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_TURNDIR7);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD9);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED40);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 12;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		case 12: //Drive straight forward toward 4th slalom right turn
		{	
			//printf("Step 12!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD10);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED60);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 13;

			}

			break;

		}
		case 13: //Execute 4th right slalom turn
		{
			//printf("Step 13!\n");

			//Arc turn forward right
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_TURNDIR8);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD11);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED40);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 14;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
		case 14: //Drive straight forward to cross End Zone line
		{	
			//printf("Step 14!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R5_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R5_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R5_FWD12);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R5_SPEED60);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 15;

			}

			break;

		}
		case 15: //All done!
		{
			//printf("Step 15!\n");

			BigRedBash();

			ResetAutoVars();

			break;

		}	

	} //End Switch

}

void XM23::AutoRight6(void)
{
	//printf("Right 6 - AutoNav Barrel Racing Path\n");

	#define R6_STEER		0.25  	//0.20 - Max steer correction

	#define R6_SPEED40	 	0.40	//0.40 
	#define R6_SPEED50		0.50	//0.50 
	#define R6_SPEED60		0.60	//0.60
	#define R6_SPEED70		0.70	//0.70
	#define R6_SPEED80		0.80	//0.80
	#define R6_SPEED90		0.90	//0.90
	#define R6_SPEED100     1.00	//1.00

	#define R6_STRAIGHTDIR   0.0	//0.0  - Straightline travel angle command

	#define R6_READYDWELL	 0.05	//Delay time waiting for arm to lower safely
	
	#define R6_FWD1			 90 	// 90 - Drive forward before 1st barrel turn

	#define R6_FWD2			 160	// 160 - Drive forward to execute 1st barrel turn
	#define R6_TURNDIR1	 	 360    // 360 - Turn Right for 1st barrel turn

	#define R6_FWD3			 30	    // 30 - Drive forward toward 2nd barrel turn

	#define R6_FWD4			 305	// 305 - Drive forward to execute 2nd barrel turn
	#define R6_TURNDIR2		 -310 	// -310 - Turn Left for 2nd barrel turn

	#define R6_FWD5			 62	    // 62 - Drive forward toward 3rd barrel turn

	#define R6_FWD6			 255	// 250 - Drive forward to execute 3rd barrel turn
	#define R6_TURNDIR3		 -229 	// -225 - Turn Left for 3rd barrel turn

	#define R6_FWD7			 190	// 180 - Drive forward to cross End Zone line
	#define R6_TURNDIR4		 4 		// 4 - Slight right turn to align with end zone

	switch (autonStep)
	{

		case 1: //Set trenchrun position mode and shift to high gear
		{
			//printf("Step 1!\n");

			autothroughtrench_sw = 1;			//Lower the arm and retract the hood

			shiftstate = SHIFTHIGH;

			autotimer->Reset();

			RightEncoder->Reset();

			gyro->Reset();

			autonStep = 2;

			break;
		}
		case 2: //Wait for arm to be ready (or watchdog timeout) 
		{				
			//printf("Step 2!\n");	

			if (autotimer->Get() > R6_READYDWELL)
			{	
				autotimer->Reset();

				autonStep = 3;
			}
		
			break;
		}		
		case 3: //Drive straight forward before 1st barrel turn
		{	
			//printf("Step 3!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R6_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R6_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R6_FWD1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R6_SPEED60);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 4;

			}

			break;

		}		
		case 4: //Execute 1st barrel turn
		{
			//printf("Step 4!\n");

			//Arc turn forward right
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R6_TURNDIR1);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R6_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R6_FWD2);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R6_SPEED50);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 5;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
	case 5: //Drive straight forward toward 2nd barrel turn
		{	
			//printf("Step 5!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R6_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R6_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R6_FWD3);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R6_SPEED60);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 6;

			}

			break;

		}
		case 6: //Execute 2nd barrel turn
		{
			//printf("Step 6!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R6_TURNDIR2);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R6_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R6_FWD4);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R6_SPEED50);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 7;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}	
	case 7: //Drive straight forward toward 3rd barrel turn
		{	
			//printf("Step 7!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R6_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R6_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R6_FWD5);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R6_SPEED60);

			if (EncoderYCtrl->isDone())	
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 8;

			}

			break;

		}		
		case 8: //Execute 3rd barrel turn
		{
			//printf("Step 8!\n");

			//Arc turn forward left
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R6_TURNDIR3);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R6_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R6_FWD6);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R6_SPEED50);

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 9;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		case 9: //Drive straight forward with slight right turn to cross End Zone line
		{	
			//printf("Step 9!\n");			

			//Drive straight forward
			GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			GyroXCtrl->setDesiredValue(R6_TURNDIR4);
			GyroXCtrl->setErrorEpsilon(2);		//3
			GyroXCtrl->setMinDoneCycles(2);     //4
			GyroXCtrl->setMaxOutput(R6_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R6_FWD7);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R6_SPEED100);  

			if (GyroXCtrl->isDone())	
			{
				gyrodonestore = 1;
			}

			if (EncoderYCtrl->isDone())
			{
				encoderdonestore = 1;
			}	
			
			if ( (gyrodonestore == 1) && (encoderdonestore == 1) )
			{
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);	

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 10;

				gyrodonestore = 0;
				encoderdonestore = 0;

			}

			break;

		}
		case 10: //All done!
		{
			//printf("Step 10!\n");

			BigRedBash();

			ResetAutoVars();

			break;

		}	

	} //End Switch
}

void XM23::AutoRight7(void)
{
	//printf("Right 7 - Drive Backward Only\n");

	#define R7_STEER		0.50	//0.50 - Max steer correction
	#define R7_SPEED1	 	0.70	//0.70 - Medium Speed
	#define R7_SPEED2		0.50	//0.50 - Slow Speed
	#define R7_SPEED3		0.90	//0.90 - Fast Speed
	#define R7_STRAIGHTDIR  0.0		//0.0  - Straightline travel angle command

	#define R7_SCOREDELAY	1.00	//1.00 - Pause Time for setting systems to smart home mode
	#define R7_REV1			-45		//-30  - Drive Backward

	switch (autonStep)
	{

		case 1: //Set home smartbutton
		{
			//printf("Step 1!\n");

			//Put the system in close (initiation line) shot position
			autospareshot_sw = 1;

			if (autotimer->Get() > R7_SCOREDELAY)
			{

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 2;

			}

			break;
		}
		case 2: //Drive backward
		{

			//printf("Step 2!\n");

			//Move forward
			GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			GyroXCtrl->setDesiredValue(R7_STRAIGHTDIR);
			GyroXCtrl->setErrorEpsilon(2);		
			GyroXCtrl->setMinDoneCycles(2);     
			GyroXCtrl->setMaxOutput(R7_STEER);

			EncoderYCtrl->setConstants(ENCODERGAIN_P, ENCODERGAIN_I, ENCODERGAIN_D);
			EncoderYCtrl->setDesiredValue(R7_REV1);
			EncoderYCtrl->setErrorEpsilon(2);
			EncoderYCtrl->setMinDoneCycles(2);
			EncoderYCtrl->setMaxOutput(R7_SPEED1);

			if (EncoderYCtrl->isDone())
			{

				GyroXCtrl->setConstants(0.0, 0.0, 0.0);

				EncoderYCtrl->setConstants(0.0, 0.0, 0.0);

				autotimer->Reset();

				RightEncoder->Reset();

				gyro->Reset();

				autonStep = 3;

			}

			break;
		}
		case 3: //All done!
		{

			//printf("Step 3!\n");

			BigRedBash();

			ResetAutoVars();

			break;

		}

	} //End Switch

}
void XM23::AutoRight8(void)
{
	//printf("Right 8 - Do Nothing\n");
	
}





void XM23::DoDriveStuff(float lyval, float lxval, float ryval, float rxval)
{

	// Retrieve and square the joystick values to desensitize them.


		leftx = XM23Lib::signSquare(lxval);
		lefty = XM23Lib::signSquare(lyval);

		rightx = XM23Lib::signSquare(rxval);
		righty = XM23Lib::signSquare(ryval);

	//	leftx = XM23Lib::signCube(lxval);
	//	lefty = XM23Lib::signCube(lyval);

	//	rightx = XM23Lib::signCube(rxval);
	//	righty = XM23Lib::signCube(ryval);

	//printf ("LXcalc %f, LYcalc %f, RXcalc %f, RYcalc %f\n", leftx, lefty, rightx, righty);
	
	//NEW - Enabling access to top speed/voltage only when left trigger is depressed
	//This code also de-rates one side if mechanically or electrically faster than the other 
	//Only enable turbo in high gear - in low, provide full power always
	
	//Derate high gear speed by the NORMDRIVESPEEDLEFT constant

	if (shiftstate == SHIFTHIGH)
	{
		/*
		if  (turbodrive_sw == 1)
		{
			lefty = lefty * TURBODRIVESPEEDLEFT;
			righty = righty * TURBODRIVESPEEDRIGHT;		
		}
		else //standard operation - do not allow full speed
		{
		*/

			//lefty = lefty * NORMDRIVESPEEDLEFT;
			//righty = righty * NORMDRIVESPEEDRIGHT;	
		
			//leftx = leftx * NORMDRIVESPEEDLEFT;
			//rightx = rightx * NORMDRIVESPEEDRIGHT;

			lefty = lefty * maxspeed;
			righty = righty * maxspeed;	
		
			leftx = leftx * maxspeed;
			rightx = rightx * maxspeed;

		//}

	}
	
	//This is the IIR tuning variable: 1.0 disables this filter	
	//leftytau = leftxtau = rightytau = rightxtau = leftjoythrottle;

	leftytau = leftxtau = rightytau = rightxtau = 0.7;    //0.8
	
	leftycalc = (leftytau * lefty) + (1 - leftytau) * oldlefty;
	oldlefty = lefty;

	leftxcalc = (leftxtau * leftx) + (1 - leftxtau) * oldleftx;
	oldleftx = leftx;
			
	rightycalc = (rightytau * righty) + (1 - rightytau) * oldrighty;
	oldrighty = righty;
			
	rightxcalc = (rightxtau * rightx) + (1 - rightxtau) * oldrightx;
	oldrightx = rightx;

	//printf ("LXcalc %f, LYcalc %f, RXcalc %f, RYcalc %f\n", leftxcalc, leftycalc, rightxcalc, rightycalc);

	if (drivestate == ARCADEMODE)
	{
		//printf("ARCADE!\n");

		if ( ( (slowmode_sw == 1) && (controlstate == JOYSTICK) ) || ( (padslowmode_sw == 1) && (controlstate == GAMEPAD) ) )
		//if ( (slowmode_sw == 1) || (padslowmode_sw == 1) )
		{
			//48
			//Cut the motor speed calculation by the SLOWRATE value (< 1.0)

			//Use right gamepad joystick for ARCADE Mode
			//LFdrive.speed = XM23Lib::limitOutput(rightycalc - rightxcalc) * SLOWRATE;
			//RFdrive.speed = -XM23Lib::limitOutput(rightycalc + rightxcalc) * SLOWRATE;

			//Use left gamepad joystick for ARCADE Mode
			LFdrive.speed = XM23Lib::limitOutput(leftycalc - leftxcalc) * SLOWRATE;
			RFdrive.speed = -XM23Lib::limitOutput(leftycalc + leftxcalc) * SLOWRATE;	
		
		}
		else
		{
			//48

			//Use right gamepad joystick for ARCADE Mode
			//LFdrive.speed = XM23Lib::limitOutput(rightycalc - rightxcalc);
			//RFdrive.speed = -XM23Lib::limitOutput(rightycalc + rightxcalc);
			
			//Use left gamepad joystick for ARCADE Mode
			LFdrive.speed = XM23Lib::limitOutput(leftycalc - leftxcalc);
			RFdrive.speed = -XM23Lib::limitOutput(leftycalc + leftxcalc);

		}

	}
	else if (drivestate == TANKMODE)
	{
		// Calculate motor outputs for tank drive.
		
		//printf("TANK!\n");

		if ( ( (slowmode_sw == 1) && (controlstate == JOYSTICK) ) || ( (padslowmode_sw == 1) && (controlstate == GAMEPAD) ) )
		//if ( (slowmode_sw == 1) || (padslowmode_sw == 1) )
		{
			//48
			
			// Cut the motor speed calculation by the SLOWRATE value (< 1.0)
			LFdrive.speed = XM23Lib::limitOutput(leftycalc) * SLOWRATE;
			RFdrive.speed = -XM23Lib::limitOutput(rightycalc) * SLOWRATE;

		
		}
		else
		{
			//48

			//LFdrive.speed = LRdrive.speed = DERATEPBOTLEFT * -XM23Lib::limitOutput(leftycalc);
			LFdrive.speed = XM23Lib::limitOutput(leftycalc);
			RFdrive.speed = -XM23Lib::limitOutput(rightycalc);
		

		}


	}

	//Gyro OR Limelight-Assisted Driving/Aiming Enabled (Drive Straight/Strafe Straight)
	if (autodrivestate == DRIVENORMALAUTO)
	{

		//printf("DRIVENORMALAUTO!\n");

		//Drive Straight Code for Teleop
		if ( ( (slowmode_sw == 1) && (controlstate == JOYSTICK) ) || ( (padslowmode_sw == 1) && (controlstate == GAMEPAD) ) )
		//if ( (slowmode_sw == 1) || (padslowmode_sw == 1) )
		{

			//printf("GYROASSIST ACTIVE!\n");

			if (slowlatch == 0)
			{
				gyro->Reset();
				gyroangle = 0.0;
				lastgyroangle = 0.0;
				slowlatch = 1;

				LightsOff();

				//printf("BLAR TEST!\n");

			}
			
			if (shiftstate == SHIFTHIGH) 
			{
				GyroXCtrl->setConstants(GYROGAINH_P, GYROGAINH_I, GYROGAINH_D);
			}
			else
			{
				GyroXCtrl->setConstants(GYROGAINL_P, GYROGAINL_I, GYROGAINL_D);
			}
			
			//Gyro Drive
			GyroXCtrl->setDesiredValue(lastgyroangle);
			steercorrect = XM23Lib::limitOutput(GyroXCtrl->calcPID((int) gyroangle));
			
			GyroXCtrl->setMaxOutput(DRIVESTRAIGHTMAXCORRECT);

			//48
			LFdrive.speed =  LFdrive.speed + steercorrect;
			RFdrive.speed =  RFdrive.speed + steercorrect;
		
		}
		else
		{
			steercorrect = 0.0;
			
			if (slowlatch == 1) //Reset gyro before leaving
			{
				gyro->Reset();
				gyroangle = 0.0;
				lastgyroangle = gyroangle;
				GyroXCtrl->setConstants(0.0, 0.0, 0.0);
				slowlatch = 0;

			} 

		}

		//TO DO - COPY THIS CODE TO AUTONOMOUSPERIODIC - AT virtual joystick PID code - enable limelight tracking in auton
		//Limelight Auto-Aim Code
		if ( (shooterstate == ON) && (targetingenabled == 1) )
		{

			//Limelight LED's ON	
			if (targetingenabled == 1) 
			{
				LightsOn();
			}
			
			if (targetdetected == 1) //A vision target has been detected - auto-aim
			{

				if (shiftstate == SHIFTHIGH) 
				{
					GyroXCtrl->setConstants(LIMEGAIN_P, LIMEGAIN_I, LIMEGAIN_D);
				}
				else
				{
					GyroXCtrl->setConstants(LIMEGAIN_P, LIMEGAIN_I, LIMEGAIN_D);
				}
				
				//Limelight Drive
				GyroXCtrl->setDesiredValue(0.0);	

				steercorrect = XM23Lib::limitOutput(GyroXCtrl->calcPID((int) -TX));
			}
			else
			{
				steercorrect = 0.0;
			}
			
			GyroXCtrl->setMaxOutput(DRIVESTRAIGHTMAXCORRECT);

			//48
			//Do not permit external drive input - force a zero-radius turn
			LFdrive.speed =  LFdrive.speed + steercorrect;
			RFdrive.speed =  RFdrive.speed + steercorrect;
			
			//LFdrive.speed = steercorrect;
			//RFdrive.speed = steercorrect;
		
		}
		else
		{
			steercorrect = 0.0;
			
			GyroXCtrl->setConstants(0.0, 0.0, 0.0);

			//Limelight LED's Off
			LightsOff();

			//DEBUG ONLY 3-13-2021
			//LightsOn();

		}

	}
	else if (autodrivestate == DRIVENORMALMAN)
	{

		//Manual Drive Mode - Disable gyro assist in teleop
		steercorrect = 0.0;
		GyroXCtrl->setConstants(0.0, 0.0, 0.0);

	}


	//SHIFTER CUTOFF CODE - Reduces Drive Motor Voltage for a short pulse while a shift transition is taking place
	#define SHIFTCUTOFFDWELL 	0.35    //0.15 Timer Preset to dwell and cutoff
	#define	SHIFTCUTOFF 		0.50	//0.70 Voltage reduction during shifting (50% of commanded)

	if (shiftlatch == 1)
	{
		if (shifttimer->Get() < SHIFTCUTOFFDWELL)
		{
			LFdrive.speed = LFdrive.speed * SHIFTCUTOFF;
			RFdrive.speed = RFdrive.speed * SHIFTCUTOFF;
		}
		else
		{
			shiftlatch = 0;
		}

	}

	LRdrive.speed = LFdrive.speed;
	LMIDdrive.speed = LFdrive.speed;
	RRdrive.speed = RFdrive.speed;
	RMIDdrive.speed = RFdrive.speed;

	driveLF->Set(LFdrive.speed);
	driveLR->Set(LRdrive.speed);
	driveLMID->Set(LMIDdrive.speed);
	
	driveRF->Set(RFdrive.speed);
	driveRR->Set(RRdrive.speed);
	driveRMID->Set(RMIDdrive.speed);
	

	//printf ("COR %f, LF %f, LR %f, RF %f, RR %f\n", steercorrect, LFdrive.speed, LRdrive.speed, RFdrive.speed, RRdrive.speed);
	//printf ("LF %f, LR %f, RF %f, RR %f\n", LFdrive.speed, LRdrive.speed, RFdrive.speed, RRdrive.speed);

}



/*
void XM23::DoArmStuff(void)
{

    //-------------------------------
    //------Arm Control Code--------
    //-------------------------------
    

	//HOMING CODE - PERMIT MANUAL HOMING WITH BUTTON PRESS - force a limit switch home and kill all arm smart buttons until sensor homing occurs!

	if ( (homearmhigh_sw == 1) && (homearmlow_sw == 0) )
	{
		ArmEncoder->Reset();
		armoffset = ARMHOMEHIGH;
		//armishome = 0;
		BigRedBash();

		//printf("MANUAL ARM HOME HIGH!\n");
	}

	if ( (homearmlow_sw == 1) && (homearmhigh_sw == 0) )
	{
		ArmEncoder->Reset();
		armoffset = ARMFRONTTRENCH;
		//armishome = 0;
		BigRedBash();

		//printf("MANUAL ARM HOME LOW!\n");
	}

	//HOMING CODE - LIMIT SWITCHES

	if ( (armatupper) && (!armatlower) )
	{
		ArmEncoder->Reset();
		armoffset = ARMHOMEHIGH;
		armishome = 1;

	}
	else if ( (armatlower) && (!armatupper) )
	{
		ArmEncoder->Reset();
		armoffset = ARMFRONTTRENCH;
		armishome = 1;
	}




	if (armlatch == 0) //Manual control of arm
	{
		
		armCtrl->setConstants(0.0, 0.0, 0.0);

		//Copilot control active

		//Lock out motion if arm has reached any soft or hard limits
		if ( (upadjustlock == 0) && ( (armatupper == 1) || (armpos >= ARMUPLIMIT) ) )
		{
			upadjustlock = 1;

			if (copadrightjoyY >= 0.1)
			{
				armspeed = 0.0;
			}

		}

		if ( (upadjustlock == 1) && (copadrightjoyY <= 0.1) && (copadrightjoyY >= -0.1) )
		{
			upadjustlock = 0;
		}

		//Manual Arm Speed Control
		if (upadjustlock == 0)
		{
			if ( (autonactive == 0) || ( (autonactive == 1) && (autodriveenable == 0) ) )
			{
				armspeed = MAXARMSPEEDMAN * XM23Lib::signCube(copadrightjoyY);
				//armspeed = MAXARMSPEEDMAN * copadrightjoyY;
			}
			else
			{
				armspeed = MAXARMSPEEDAUTO * autopadrightjoyY;
			}

		}

	}
	else //Automatic Arm Control Active
	{

		//Bump Adjustment of Arm Position in Smart Button Mode

		if ( (adjustlock == 1) && (copadrightjoyY >= LOWERADJUSTDEAD) && (copadrightjoyY <= UPPERADJUSTDEAD) )
		{
			adjustlock = 0;
		}

		if ( (copadrightjoyY > UPPERADJUSTDEAD) && (adjustlock == 0) )
		{

			//Adjust the commanded value

			armcmd = armcmd + BUMPCMDUP;
			armCtrl->setDesiredValue(armcmd);

			adjustlock = 1;

			//printf("Blar Up!\n");
			
		}
		else if ( (copadrightjoyY < LOWERADJUSTDEAD) && (adjustlock == 0) )
		{

			//Adjust the commanded value

			armcmd = armcmd - BUMPCMDDOWN;
			armCtrl->setDesiredValue(armcmd);

			adjustlock = 1;

			//printf("Blar Down!\n");

		}

		armspeed = armCtrl->calcPID(armpos);

	}


	//Max Speed Protection

	if (armlatch == 1)
	{
	    if (armspeed > MAXARMSPEEDAUTO)
	    {
	    	armspeed = MAXARMSPEEDAUTO;
	    }
	    else if (armspeed < -MAXARMSPEEDAUTO)
	    {
	    	armspeed = -MAXARMSPEEDAUTO;
	    }

	}
	else
	{
		if (armspeed > MAXARMSPEEDMAN)
		{
			armspeed = MAXARMSPEEDMAN;
		}
		else if (armspeed < -MAXARMSPEEDMAN)
		{
			armspeed = -MAXARMSPEEDMAN;
		}

	}

	//Soft Arm Limits from Arm Angle Sensor

	if (armpos >= ARMUPLIMIT) 	//Arm angle increases as arm moves up
	{

		//printf("UPPER SOFT LIMIT\n");

		if (armspeed > NEUTRAL) armspeed = NEUTRAL;

	}
	else if (armpos <= ARMDOWNLIMIT)
	{

		//printf("LOWER SOFT LIMIT\n");

		if (armspeed < NEUTRAL) armspeed = NEUTRAL;

	}

	//Speed Limits if Operating Near Ends of Travel

	if (armpos > (ARMUPLIMIT - SLOWARMTHRESH) )
	{
		//Limit manual speed when traveling close to upper limit and toward the limit
		if (armspeed > 0.0) armspeed = armspeed * DERATEARMSPEEDUP;
	}

	if (armpos < (ARMDOWNLIMIT + SLOWARMTHRESH) )
	{
		//Limit manual speed when traveling close to lower limit and toward the limit
		if (armspeed < 0.0) armspeed = armspeed * DERATEARMSPEEDDOWN;
	}

	if (armpos < (ARMDOWNLIMIT + SLOWESTARMTHRESH) )
	{
		//Limit manual speed when traveling closest to lower limit and toward the limit 
		if (armspeed < 0.0) armspeed = armspeed * DERATEARMSPEEDDOWNSLOW;

		//Limit manual speed when traveling closest to lower limit and away from limit
		if (armspeed > 0.0) armspeed = armspeed * DERATEARMSPEEDUPSLOW;	
	}

	//Hard Arm Limits from Limit Switches/Proxes

	if (armatupper) 	//Arm has hit the upper limit switch
	{

		//printf("UPPER HARD LIMIT\n");

		if (armspeed > NEUTRAL) armspeed = NEUTRAL;

	}
	else if (armatlower)
	{

		//printf("LOWER HARD LIMIT\n");

		if (armspeed < NEUTRAL) armspeed = NEUTRAL;

	}




	//-----------SEND THE ARM MOTOR COMMAND TO THE SPEED CONTROLLER-----------

	if ( (armlatch == 1) && (armspeed > 0.0) )
	{
		//arm->Set(ARMUPBIAS + armspeed);
		arm->Set(armspeed);
	}
	else
	{
		arm->Set(armspeed);
	}
	
	if ( (armlatch == 1) && (abs(armcmd - armpos) <= ARMOKTHRESH) )
	{
		armposok = 1;

		//NEW 2018 - when you get into range, kill the arm smart button
		//SmartReset();

	}
	else
	{
		armposok = 0;
	}


} //DoArmStuff()

*/

void XM23::DoClimbStuff(void)
{
	//**********************************
	// Climber Control
	//**********************************

	//Climber Safety Unlock Code
	if ( (climbersafety_sw == 1) && (climberetract_sw == 1) && (climberextend_sw == 1) ) 
	{
		climberenabled = 1;

		autoclimbmode_sw = 1;   //Initiate climb mode smartbutton (retract everything, 0 RPM shooter command)
	}

	//HOMING CODE - LIMIT SWITCHES

	if ( (climbextended) && (!climbretracted) )
	{
		climber->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		climboffset = CLIMBERMAX;
		climbishome = 1;

	}                              
	//Use climbersafety to reset the low home position (if robot powered on with climber not all the way down)
	else if ( ( (climbersafety_sw == 1) && (climberenabled == 1) && (climberextend_sw == 0) && (climberetract_sw == 0) ) || ( (climbersafety_sw == 0) && (climbretracted) && (!climbextended) ) )
	{
		climber->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		climboffset = CLIMBERHOME;
		climbishome = 1;
	}


	if (climblatch == 0) //Manual control of climber
	{

		climbCtrl->setConstants(0.0, 0.0, 0.0);

		/*
		//Lock out motion if climber has reached any soft or hard limits
		if ( (climberetractadjustlock == 0) && ( (climbretracted == 1) || (climbpos <= CLIMBERMIN) ) )
		{
			climberetractadjustlock = 1;

			if (climberetract_sw == 1)
			{
				climbspeed = 0.0;
			}

		}

		if ( (climberetractadjustlock == 1) && (climberetract_sw == 0) )
		{
			climberetractadjustlock = 0;
		}
		*/

		//Manual Climber Speed Control
		//if (climberetractadjustlock == 0)
		//{
			if ( ( (climberetract_sw == 1) || (autoclimbretract_sw == 1) ) && (climberextend_sw == 0) && (autoclimbextend_sw == 0) && (climberenabled == 1) ) 
			{
				climbstate = CLIMBRETRACT;

				autohoodup_sw = 1;  //Lock the pawl with the hood, enabling the climber ratcheting action
			}
			else if ( ( (climberextend_sw == 1) || (autoclimbextend_sw == 1) ) && (climberetract_sw == 0) && (autoclimbretract_sw == 0) && (climberenabled == 1) ) 
			{
				climbstate = CLIMBEXTEND;

				autohooddown_sw = 1;  //Unlock the pawl with the hood, permitting free movement of the climber to reach high
			}
			else  //Motor is off
			{
				climbstate = CLIMBOFF;
			}	
		
			if (climbstate == CLIMBRETRACT)
			{
				if (climbersafety_sw == 0)
				{
					climbspeed = CLIMBRETRACTSPEED;
				}
				else
				{
					climbspeed = CLIMBRETRACTSAFESPEED;
				}
				
			}
			else if (climbstate == CLIMBEXTEND)
			{
				if (climbersafety_sw == 0)
				{
					climbspeed = CLIMBEXTENDSPEED;
				}
				else
				{
					climbspeed = CLIMBEXTENDSAFESPEED;
				}
				
			}
			else // climbstate == CLIMBOFF
			{
				climbspeed = 0.0;
			}

		//}

	}
	else //Automatic Climber Control Active
	{

		climbspeed = climbCtrl->calcPID(climbpos);

		/*
		//Manual Adjustment of Climber Position in Smart Button Mode
		if ( (climberetract_sw == 0) && (climberextend_sw == 0) )
		{
			
			if (climberadjustlock == 1)
			{

				climbcmd = climbpos;
				climbCtrl->setDesiredValue(climbcmd);

				climberadjustlock = 0;

			}

		}

		if (climberextend_sw == 1)
		{

			//Adjust the commanded value

			climbcmd = climbcmd + CLIMBBUMPCMDUP;
			climbCtrl->setDesiredValue(climbcmd);

			//ALTERNATE - manual adjustment
			//climbspeed = MAXCLIMBSPEEDMANTWEAKUP;

			climberadjustlock = 1;

			//printf("Blar Up!\n");
			
		}
		else if (climberetract_sw == 1)
		{

			//Adjust the commanded value

			climbcmd = climbcmd - CLIMBBUMPCMDDOWN;
			climbCtrl->setDesiredValue(climbcmd);

			//ALTERNATE - manual adjustment
			//armspeed = MAXCLIMBSPEEDMANTWEAKDOWN;

			climberadjustlock = 1;

			//printf("Blar Down!\n");

		}
		*/

	}

	//Soft climber Limits from climber encoder


	
	if (climbpos >= CLIMBERUPLIMIT) 	//climber distance increases as climber moves up
	{

		//printf("UPPER SOFT LIMIT\n");

		if (climbspeed > NEUTRAL) climbspeed = NEUTRAL;

	}
	//Override if climber safety button is pressed so you can drive the arm down slowly if not at home at startup
	else if ( (climbersafety_sw == 0) && (climbpos <= CLIMBERDOWNLIMIT) ) 
	{

		//printf("LOWER SOFT LIMIT\n");

		if (climbspeed < NEUTRAL) climbspeed = NEUTRAL;

	}

	//Speed Limits if Operating Near Ends of Travel

	if (climbpos > (CLIMBERUPLIMIT - SLOWCLIMBERTHRESHUP) )
	{
		//Limit manual speed when traveling close to upper limit and toward the limit
		if (climbspeed > 0.0) climbspeed = climbspeed * DERATECLIMBSPEEDUP;
	}

	if (climbpos < (CLIMBERDOWNLIMIT + SLOWCLIMBERTHRESHDOWN) )
	{
		//Limit manual speed when traveling close to lower limit and toward the limit
		if (climbspeed < 0.0) climbspeed = climbspeed * DERATECLIMBSPEEDDOWN;
	}

	/*
	//Hard climber Limits from Limit Switches/Proxes

	if (climbretracted) 	//climber has hit the down limit switch
	{

		//printf("LOWER HARD LIMIT\n");

		if (climbspeed < NEUTRAL) climbspeed = NEUTRAL;

	}
	else if (climbextended)
	{

		//printf("UPPER HARD LIMIT\n");

		if (climbspeed > NEUTRAL) climbspeed = NEUTRAL;

	}
	*/


	//-----------SEND THE CLIMBER MOTOR COMMAND TO THE SPEED CONTROLLER-----------
	
	//Debug Print
	/*
	if (++_loops >= 10) 
	{
		_loops = 0;
		_sb.append("Setting climber speed to ");
		_sb.append(std::to_string(climbspeed));
		printf("%s\n",_sb.c_str());
		_sb.clear();

	}
	*/

	climber->Set(ControlMode::PercentOutput, climbspeed);
	
	if (abs(climbcmd - climbpos) <= CLIMBEROKTHRESH)
	{
		climbposok = 1;
	}
	else
	{
		climbposok = 0;
	}

} //DoClimbStuff()


void XM23::DoShooterStuff(void)
{

	//Turn the shooter on or off
	if ( ( (shooteron_sw == 1) || (autoshooteron_sw == 1) ) && (!shooteroff_sw) && (!autoshooteroff_sw) ) // && (cshooteroff_sw == 0) )
	{
		shooterstate = ON;

		if (autoshooteron_sw == 1) autoshooteron_sw = 0;

		//printf("SHOOTER ON!!!\n");

	}
	else if ( ( (shooteroff_sw == 1) || (autoshooteroff_sw == 1) ) && (!shooteron_sw) && (!autoshooteron_sw) ) // || (cshooteroff_sw == 1) )
	{

		if (autoshooteroff_sw == 1) autoshooteroff_sw = 0;

		shooterstate = OFF;

		//Reset the Smart Button whenever you turn the shooter off
		//BigRedBash();

		//printf("SHOOTER OFF!!!\n");

	}

/*
	//Select automatic or manual speed control
	if ( (autoshooter_sw == 1) && (manualshooter_sw == 0) )
	{
		shootermode = AUTOSHOOT;
	}
	else if ( (manualshooter_sw == 1) && (autoshooter_sw == 0) )
	{
		shootermode = MANUALSHOOT;
	}
*/

	if (shootermode == MANUALSHOOT)
	{

		if (shooterstate == ON)
		{

			if (autoshooteron_sw == 1) autoshooteron_sw = 0;

			//Right Joystick Throttle Shooter Speed Adjust
			
			//DEBUG ONLY
			//shooterspeed = MINSHOOTERSPEEDMAN + ( (MAXSHOOTERSPEEDMAN - MINSHOOTERSPEEDMAN) * ( (1 + delayadjust) / 2) );

			//shooterspeed = shootercmd/MAXRPM;   //Feed forward without PID compensation - manual speed control (what about voltage compensation?)

			//LightsOn();

		}
		else
		{
			shooterspeed = 0.0;

			//LightsOff();

		}
	}
	else //shootermode == AUTOSHOOT
	{

		if (shooterstate == ON)
		{

			if (autoshooteron_sw == 1) autoshooteron_sw = 0;

			//Manual Adjustment of Shooter Wheel Speeds in Smart Button Mode

			if ( (sadjustlock == 1) && (copadleftjoyY >= LOWERADJUSTDEAD) && (copadleftjoyY <= UPPERADJUSTDEAD) )
				{
					sadjustlock = 0;
				}

				if ( (copadleftjoyY > UPPERADJUSTDEAD) && (sadjustlock == 0) )
				{
					shootercmd = shootercmd + BUMPSPEEDUP;
					sadjustlock = 1;

					//Adjust the commanded value

					//printf("Blar Up!\n");

					shooterCtrl->setDesiredValue(shootercmd);
				}
				else if ( (copadleftjoyY < LOWERADJUSTDEAD) && (sadjustlock == 0) )
				{
					shootercmd = shootercmd - BUMPSPEEDDOWN;
					sadjustlock = 1;

					//Adjust the commanded value

					//printf("Blar Down!\n");

					shooterCtrl->setDesiredValue(shootercmd);
				}

			//Disabled due to smart button commanded speeds
			//shootercmd = MINRPM + ( (MAXRPM - MINRPM) * ( (1 + speedadjust) / 2) );
			//shooterCtrl->setDesiredValue(shootercmd);

			shooterCtrl->setConstants(SHOOTERGAIN_P, SHOOTERGAIN_I, SHOOTERGAIN_D);
			shooterCtrl->setErrorEpsilon(10);
			shooterCtrl->setMinDoneCycles(0);
			shooterCtrl->setMaxOutput(MAXSHOOTERSPEEDMAN);

			shooterspeed = shootercmd/MAXRPM + shooterCtrl->calcPID(shooterRPM);

			//LightsOn();

		}
		else
		{
			shooterspeed = 0.0;
			shooterCtrl->setConstants(0.0, 0.0, 0.0);

			//LightsOff();
			
		}

	}

			//Max Speed Protection - Active for either Manual or Closed-Loop

		    if (shooterspeed > MAXSHOOTERSPEEDMAN)
		    {
		    	shooterspeed = MAXSHOOTERSPEEDMAN;
		    }
		    else if (shooterspeed < 0.0)
		    {
		    	shooterspeed = 0.0;
		    }

			if ( (shooterstate == OFF) && (ballfeedstate == BALLFEEDLOAD) )
			{

				shooterspeed = SHOOTERREVSPEED;  //RUN THE SHOOTER BACKWARDS WHILE LOADING TO PREVENT BALLS IN NO MAN'S LAND

			}
			

			shootertop->Set(ControlMode::PercentOutput, shooterspeed);




			if ( (shooterstate == ON) && ( ( (shootermode == AUTOSHOOT) && (abs(shootercmd - shooterRPM) <= SPEEDOKTHRESH) ) || (shootermode == MANUALSHOOT) ) )
			{
				speedok = 1;
			}
			else
			{
				speedok = 0;
			}

			if ( (shooterstate == ON) && (speedok == 1) && (targetdetected == 1) ) //ADD OTHER CONSTRAINTS HERE
			{
				oktoshoot = 1;
			}
			else
			{
				oktoshoot = 0;
			}

} //DoShooterStuff()

void XM23::DoShiftStuff(void)
{

    //**************************
	// Shifter Control
    //**************************

	if ( (shifthigh_sw || padshifthigh_sw) && ( (!shiftlow_sw) && (!padshiftlow_sw) ) )
	{
		if ( (shiftlatch == 0) && (shiftstate == SHIFTLOW) ) //This is a transition - start the shift timer and cut voltage for a short pulse
		{
			shiftlatch = 1;

			shifttimer->Reset();
		}

		shiftstate = SHIFTHIGH;

	}
	
	
	if ( (shiftlow_sw || padshiftlow_sw) && ( (!shifthigh_sw) && (!padshifthigh_sw) ) )
	{
		if ( (shiftlatch == 0) && (shiftstate == SHIFTHIGH) ) //This is a transition - start the shift timer and cut voltage for a short pulse
		{
			shiftlatch = 1;

			shifttimer->Reset();
		}

		shiftstate = SHIFTLOW;
	

	}

	

	// Control the shifter
	if (shiftstate == SHIFTHIGH)
	{
		shifter->Set(HIGHGEAR);
		
		highgearon = 1;
		lowgearon = 0;

	}
	else if (shiftstate == SHIFTLOW)
	{
		shifter->Set(LOWGEAR);

		highgearon = 0;
		lowgearon = 1;

	}

}

void XM23::DoOtherStuff(void)
{

    //
	//Aiming Light Control
    //

/*
	//if ( (plighton_sw == 1) && (plightoff_sw == 0) )
	if (shooterstate == ON)
	{
		
		light->Set(LIGHTON);	//Turn the aiming light on

	}
	//else if ( (plightoff_sw == 1) && (plighton_sw == 0) )
	else
	{
		
		light->Set(LIGHTOFF);	//Turn the aiming light off
		
	}
*/



	//**************************
	// Shooter Cage Tllt Control
	//**************************

	//DEBUG 2021 added blinkinoverride as a tilt down trigger for pilot-only runs in @home hyperdrive challenge
	if ( ( (padarmdown_sw == 1) || (blinkinoverride_sw == 1) || (ptiltdown_sw == 1) || (autotiltdown_sw == 1) ) && (!ptiltup_sw) && (!autotiltup_sw) ) 
	{
		tiltstate = TILTDOWN;

		if (autotiltdown_sw == 1) autotiltdown_sw = 0;

	}
	else if ( ( (ptiltup_sw == 1) || (autotiltup_sw == 1) ) && (!padarmdown_sw) && (!blinkinoverride_sw) && (!ptiltdown_sw) && (!autotiltdown_sw) ) 
	{
		tiltstate = TILTUP;

		if (autotiltup_sw == 1) autotiltup_sw = 0;

	}

	if (tiltstate == TILTUP)
	{
		//printf("TILT UP!\n");

		tilt->Set(TILTBACK);

	}
	else //tiltstate == TILTDOWN
	{
		//printf("TILT DOWN!\n");

		tilt->Set(TILTFORWARD);

	}


	//**************************
	// Shooter Hood Control
	//**************************

	if ( (autohooddown_sw == 1) && (autohoodup_sw == 0) ) 
	{
		hoodstate = HOODDOWN;

		if (autohooddown_sw == 1) autohooddown_sw = 0;

	}
	else if ( (autohoodup_sw == 1) && (autohooddown_sw == 0) )
	{
		hoodstate = HOODUP;

		if (autohoodup_sw == 1) autohoodup_sw = 0;

	}

	if (hoodstate == HOODUP)
	{
		//printf("HOOD UP!\n");

		hood->Set(SHOOTFAR);

	}
	else //hoodstate == HOODDOWN
	{
		//printf("HOOD DOWN!\n");

		hood->Set(SHOOTCLOSE);

	}


	//******************************
	// Intake Deploy Control
	//******************************


	if ( ( (intakedeploy_sw == 1) || (autointakedeploy_sw == 1) ) && (!intakeretract_sw) && (!autointakeretract_sw) )
	{
				
		intakedeploystate = DEPLOYINTAKE;

		if (autointakedeploy_sw == 1) autointakedeploy_sw = 0;
 
	}
	else if ( ( (intakeretract_sw == 1) || (autointakeretract_sw == 1) ) && (!intakedeploy_sw) && (!autointakedeploy_sw) )
	{
		
		intakedeploystate = RETRACTINTAKE;

		if (autointakeretract_sw == 1) autointakeretract_sw = 0;
 
	}


	if (intakedeploystate == DEPLOYINTAKE)
	{
		//printf("INTAKE OUT!\n");

		intakedeploy->Set(INTAKEOUT);

	}
	else //intakedeploystate == RETRACTINTAKE
	{
		//printf("INTAKE IN!\n");

		intakedeploy->Set(INTAKEIN);
	}



	//**************************
	// Climb Lock Solenoid Control
	//**************************

	//THIS IS AN EXAMPLE OF AN ALTERNATE ACTION CONTROL

	if ( (climblocklatch == 1) && (blinkinoverride_sw == 0) ) climblocklatch = 0;

	if ( ( (1 == 0) || (autoclimblockon_sw == 1) ) && (climblockstate == CLIMBLOCKOFF) && (climblocklatch == 0) )
	{
		climblockstate = CLIMBLOCKON;
		climblocklatch = 1;

	}
	else if ( ( (1 == 0) || (autoclimblockoff_sw == 1) ) && (climblockstate == CLIMBLOCKON) && (climblocklatch == 0) ) 
	{
		climblockstate = CLIMBLOCKOFF;
		climblocklatch = 1;

	}

	if (climblockstate == CLIMBLOCKON)
	{
		//printf("CLIMB LOCK ON!\n");

		climblock->Set(LOCKON);

		climblockon = 1;

	}
	else //climblockstate == CLIMBLOCKOFF
	{
		//printf("CLIMB LOCK OFF!\n");

		climblockon = 0;

		climblock->Set(LOCKOFF);

	}


//BEGIN MOTOR CONTROL


	//**********************************
	// Power Cell Intake Control
	//**********************************

	if ( ( (psuck_sw == 1) || (autosuck_sw == 1) ) && (pspit_sw == 0) && (autospit_sw == 0) )
	//if ( ( (cballfeedshoot_sw == 1) || (psuck_sw == 1) || (autosuck_sw == 1) ) && (pspit_sw == 0) && (autospit_sw == 0) )
	{
		intakestate = SUCK;   

		//5-6-2021 RESTORED
		autointakedeploy_sw = 1;  //Automatically deploy the intake whenever the pilot collects
		//if (!cballfeedshoot_sw) autointakedeploy_sw = 1;  //Automatically deploy the intake whenever the pilot collects

	}
	else if ( ( (pspit_sw == 1) || (autospit_sw == 1) ) && (psuck_sw == 0) && (autosuck_sw == 0) )
	{
		intakestate = SPIT;

	}
	else //Intake is either trying to hold onto power cells (prairie doggin' it) or completely off
	{

		intakestate = INTAKEOFF;

	}


	if (intakestate == SUCK)
	{
		intakespeed = SUCKSPEED;
	}
	else if (intakestate == SPIT)
	{
		intakespeed = SPITSPEED;
	}
	else if (intakestate == HOLD)
	{
		intakespeed = HOLDSPEED;
	}
	else // intakestate == INTAKEOFF
	{
		intakespeed = 0.0;
	}


	intake->Set(intakespeed);

	/*
	//Auto Cargo Hold Reset Timer
	#define CARGORESETDWELL   0.60    //0.75 seconds

	if ( (ballholdlatch == 1)  && (ballpresentlow == 1) ) cargoholdtimer->Reset();   //Reset the cargo hold reset timer whenever auto hold is enabled and cargo is present

	if ( (ballholdlatch == 1) && (ballpresentlow == 0) && cargoholdtimer->Get() >= CARGORESETDWELL) 
	{

		ballholdlatch = 0;

		blinkincode = FIRELARGE;
		blinkintimer->Reset();

	}
	*/

	//**********************************
	// Ball Feed Conveyor Control
	//**********************************

	if ( ( (cballfeedload_sw == 1) || (autoballfeedload_sw == 1) ) && (!cballfeedeject_sw) && (!cballfeedshoot_sw) && (!autoballfeedeject_sw) && (!autoballfeedshoot_sw) ) 
	{
		ballfeedstate = BALLFEEDLOAD;
	}
	else if ( ( (cballfeedshoot_sw == 1) || (autoballfeedshoot_sw == 1) ) && (!cballfeedeject_sw) && (!cballfeedload_sw) && (!autoballfeedeject_sw) && (!autoballfeedload_sw) ) 
	{
		ballfeedstate = BALLFEEDSHOOT;
	}
	else if ( ( (cballfeedeject_sw == 1) || (autoballfeedeject_sw == 1) ) && (!cballfeedshoot_sw) && (!cballfeedload_sw) && (!autoballfeedshoot_sw) && (!autoballfeedload_sw) )
	{
		ballfeedstate = BALLFEEDEJECT;
	}
	else // ballfeedstate == BALLFEEDOFF
	{
		ballfeedstate = BALLFEEDOFF;
	}


	if (ballfeedstate == BALLFEEDLOAD)
	{
		
		if (autoballfeedload_sw == 0)
		{
			ballfeedspeed = BALLFEEDLOADSPEED;
		}
		else
		{
			ballfeedspeed = BALLFEEDLOADSPEEDSLOW;
		}

	}
	else if (ballfeedstate == BALLFEEDSHOOT)
	{
		ballfeedspeed = BALLFEEDSHOOTSPEED;
	}
	else if (ballfeedstate == BALLFEEDEJECT)
	{
		if (autoballfeedeject_sw == 0)
		{
			ballfeedspeed = BALLFEEDEJECTSPEED;
		}
		else
		{
			ballfeedspeed = BALLFEEDEJECTSPEEDSLOW;
		}
	}
	else // ballfeedstate == BALLFEEDOFF
	{
		ballfeedspeed = 0.0;
	}

	ballfeed->Set(ballfeedspeed);


	//***************************************
	// R2-D2 Color Wheel Manipulator Control
	//***************************************

	if ( ( (r2d2manual_sw == 1) || (autor2d2manual_sw == 1) ) && (!r2d2color_sw) && (!autor2d2color_sw) ) 
	{
		r2d2state = R2D2MANUAL;

	}
	else if ( ( (r2d2color_sw == 1) || (autor2d2color_sw == 1) ) && (!r2d2manual_sw) && (!autor2d2manual_sw) ) 
	{
		r2d2state = R2D2COLORPICK;
	}
	else if (1 == 0)
	{
		r2d2state = R2D2STAGE2;
	}
	else // r2d2state == R2D2OFF
	{
		r2d2state = R2D2OFF;
	}


	if (r2d2state == R2D2MANUAL)
	{
		r2d2speed = R2D2SPEED;
	}
	else if (r2d2state == R2D2COLORPICK)
	{
		r2d2speed = R2D2SPEED;
	}
	else if (r2d2state == R2D2STAGE2)
	{
		r2d2speed = R2D2SPEED;
	}
	else // r2d2state == R2D2OFF
	{
		r2d2speed = 0.0;
	}

	//r2d2->Set(r2d2speed);

	
}



void XM23::DoCameraStuff(void)
{

	/*
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture("cam0",0);
	camera.SetResolution(320, 240);
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
	cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 320, 240);
	cv::Mat source;
	cv::Mat output;
	while(true) {
		cvSink.GrabFrame(source);
		cvtColor(source, output, cv::COLOR_BGR2GRAY);
		outputStreamStd.PutFrame(output);
	}
	*/

}

void XM23::DriveNeutral(void)
{
	// Drive System
	driveLF->Set(0.0);
	driveLR->Set(0.0);
	driveRF->Set(0.0);
	driveRR->Set(0.0);
	driveRMID->Set(0.0);
	driveLMID->Set(0.0);

}

void XM23::DoSmartStuff(void)
{

/*****************************************************************************
**----------------------- SMARTBUTTON (TM) LOGIC -----------------------**
*****************************************************************************/

	if ( (sbexecute_sw == 1) && (smartbuttonlock == 0) ) 
	{
		if (joyrightstate == JOYUP)
		{
			//printf("LONG PASSING MODE\n");
			passmode_sw = 1;
		}
		else if (joyrightstate == JOYNEUTRAL)
		{
			//printf("HOME INITIATION SHOT\n");
			homeshot_sw = 1;
		}
		else if (joyrightstate == JOYDOWN)
		{
			//printf("LOW BRIDGE THROUGH TRENCH\n");
			throughtrench_sw = 1;
		}	
		else if (joyrightstate == JOY_UPLEFT)
		{
			//printf("SPARE SHOT OPTION - 1/3 FIELD\n");
			spareshot_sw = 1;	
		}	
		else if (joyrightstate == JOYLEFT)
		{
			//printf("FRONT TRENCH SHOT\n");
			fronttrench_sw = 1;
		}	
		else if (joyrightstate == JOY_DOWNLEFT)
		{
			//printf("BEHIND CONTROL PANEL SHOT\n");
			farshot_sw = 1;
		}	
		else if (joyrightstate == JOY_UPRIGHT)
		{
			//printf("CLOSE WALL SHOT\n");
			wallshot_sw = 1;
		}	
		else if (joyrightstate == JOYRIGHT)
		{
			//printf("ENDGAME CLIMB MODE\n");
			climbmode_sw = 1;
		}		
		else if (joyrightstate == JOY_DOWNRIGHT)
		{
			//printf("LOW PORT SHOT\n");
			lowport_sw = 1;
		}		
	}
	else
	{
		wallshot_sw = 0;
		fronttrench_sw = 0;
		throughtrench_sw = 0;
		passmode_sw = 0;
		homeshot_sw = 0;
		farshot_sw = 0;
		climbmode_sw = 0;
		spareshot_sw = 0;
		lowport_sw = 0;
	}


	if (bigred_sw == 1)
	{
	 
		//Reset All SmartButton Routines and their Status LED's
		//Reset all "virtual" SmartButtons
	
		//printf("BIG RED!!!\n");
		
		BigRedBash();	//Perform the Smart Button Reset

	}
	else
	{
		
		bigredrelease = 0;

		if ( ( (passmode_sw + homeshot_sw + farshot_sw + climbmode_sw + spareshot_sw + lowport_sw + wallshot_sw + fronttrench_sw + throughtrench_sw + cthroughtrench_sw + cwall_sw + cspareshot_sw + ctrenchfront_sw + cfarshot_sw) == 1 ) || ( (autopassmode_sw + autohomeshot_sw + autofarshot_sw + autoclimbmode_sw + autospareshot_sw + autolowport_sw + autowallshot_sw + autofronttrench_sw + autothroughtrench_sw) == 1 ) ) 
		{
			//A manual or "virtual autonomous" Smart Button has been pressed
			
			if ( (passmode_sw == 1) || (autopassmode_sw == 1) )	//Long distance ammo transfer across midfield
			{

				if (sbpassmodeactive == 0)
				{
					
					SmartReset();  //Reset previous smart function and record last smart function state

					shootercmd = RPMPASSMODE;
					shooterCtrl->setDesiredValue(shootercmd);

					//Set the tilt up
					autotiltup_sw = 1;

					//Set the hood down (high trajectory shot)
					autohooddown_sw = 1;

					//Turn the shooter off
					autoshooteroff_sw = 1;

					//Turn off vision targeting
					targetingenabled = 0;

					//armcmd = ARMPASSMODE;
					//armCtrl->setDesiredValue(armcmd);
					//armCtrl->setConstants(ARMGAIN_P, ARMGAIN_I, ARMGAIN_D);
					//armCtrl->setErrorEpsilon(1.0);
					//armCtrl->setMinDoneCycles(0);
					//armCtrl->setMaxOutput(MAXARMSPEEDAUTO);

					//LatchSmartValues(); //ALWAYS DISABLED

					//armlatch = 1;

					sbpassmodeactive = 1;
	   
					autopassmode_sw = 0;

					//passmode_LED = 1;
				}
			
			}
			else if ( (homeshot_sw == 1) || (autohomeshot_sw == 1) )	//Starting position initiation line shot
			{

				if (sbhomeshotactive == 0)
				{

					SmartReset();  //Reset previous smart function and record last smart function state

					shootercmd = RPMHOMESHOT;
					shooterCtrl->setDesiredValue(shootercmd);

					//Set the tilt up
					autotiltup_sw = 1;

					//Set the hood up (regular shot)
					autohoodup_sw = 1;

					//Set the intake deploy up
					autointakeretract_sw = 1;

					//Turn the shooter off
					autoshooteroff_sw = 1;

					//Turn on vision targeting
					targetingenabled = 1;

					//armcmd = ARMHOMESHOT;
					//armCtrl->setDesiredValue(armcmd);
					//armCtrl->setConstants(ARMGAIN_P, ARMGAIN_I, ARMGAIN_D);
					//armCtrl->setErrorEpsilon(1.0);
					//armCtrl->setMinDoneCycles(0);
					//armCtrl->setMaxOutput(MAXARMSPEEDAUTO);

			   		//LatchSmartValues();

					//armlatch = 1;

		    		sbhomeshotactive = 1;

					autohomeshot_sw = 0;

					//homeshot_LED = 1;
		 		}

			}
			else if ( (farshot_sw == 1) || (cfarshot_sw == 1) || (autofarshot_sw == 1) )		//Long shot behind the control panel
			{

				if (sbfarshotactive == 0)
				{
					SmartReset();  //Reset previous smart function and record last smart function state

					shootercmd = RPMFARSHOT;
					shooterCtrl->setDesiredValue(shootercmd);

					//Set the tilt up
					autotiltup_sw = 1;

					//Set the hood up (regular shot)
					autohoodup_sw = 1;

					//Set the intake deploy up
					autointakeretract_sw = 1;

					//Turn the shooter off
					autoshooteroff_sw = 1;

					//Turn on vision targeting
					targetingenabled = 1;

					//armcmd = ARMFARSHOT;
					//armCtrl->setDesiredValue(armcmd);
					//armCtrl->setConstants(ARMGAIN_P, ARMGAIN_I, ARMGAIN_D);
					//armCtrl->setErrorEpsilon(0.5);
					//armCtrl->setMinDoneCycles(0);
					//armCtrl->setMaxOutput(MAXARMSPEEDAUTO);

					//LatchSmartValues(); 
				
					//armlatch = 1;

					sbfarshotactive = 1;

					autofarshot_sw = 0;
					
					//farshot_LED = 1;
				}
     
			}
			else if ( (climbmode_sw == 1) || (autoclimbmode_sw == 1) )		//Setup for climbing
			{
	
				if (sbclimbmodeactive == 0)
				{
					SmartReset();  //Reset previous smart function and record last smart function state

					shootercmd = RPMCLIMBMODE;
					shooterCtrl->setDesiredValue(shootercmd);

					//Set the tilt up
					autotiltup_sw = 1;

					//Set the hood DOWN (unlock the climber pawl before climbing)
					autohooddown_sw = 1;

					//Set the intake deploy up
					autointakeretract_sw = 1;

					//Turn the shooter off
					autoshooteroff_sw = 1;

					//Turn off vision targeting
					targetingenabled = 0;

					//armcmd = ARMCLIMBMODE;
					//armCtrl->setDesiredValue(armcmd);
					//armCtrl->setConstants(ARMGAIN_P, ARMGAIN_I, ARMGAIN_D);
					//armCtrl->setErrorEpsilon(1.0);
					//armCtrl->setMinDoneCycles(0);
					//armCtrl->setMaxOutput(MAXARMSPEEDAUTO);

			   		//LatchSmartValues(); 

					//armlatch = 1;

					sbclimbmodeactive = 1;

					autoclimbmode_sw = 0;

					//climbmode_LED = 1;
		 		}
		 
			}
			else if ( (spareshot_sw == 1) || (cspareshot_sw == 1) || (autospareshot_sw == 1) ) 	//Spare shot position (1/3 field for 2021 @ Home)
			{

				if (sbspareshotactive == 0)
				{
					SmartReset();  //Reset previous smart function and record last smart function state

					shootercmd = RPMSPARESHOT;
					shooterCtrl->setDesiredValue(shootercmd);

					//Set the tilt up
					autotiltup_sw = 1;

					//Set the hood up (regular shot)
					autohoodup_sw = 1;

					//Set the intake deploy up
					autointakeretract_sw = 1;

					//Turn the shooter off
					autoshooteroff_sw = 1;
					
					//Turn on vision targeting
					targetingenabled = 1;

					//armcmd = ARMSPARESHOT;
					//armCtrl->setDesiredValue(armcmd);
					//armCtrl->setConstants(ARMGAIN_P, ARMGAIN_I, ARMGAIN_D);
					//armCtrl->setErrorEpsilon(1.0);
					//armCtrl->setMinDoneCycles(0);
					//armCtrl->setMaxOutput(MAXARMSPEEDAUTO);

					//LatchSmartValues(); 
	
					//armlatch = 1;

					sbspareshotactive = 1;

					autospareshot_sw = 0;

					//spareshot_LED = 1;
		 		}
		 
			}
			else if ( (lowport_sw == 1) || (autolowport_sw == 1) )		//Score in the low port
			{
	
				if (sblowportactive == 0)
				{
					SmartReset();  //Reset previous smart function and record last smart function state

					shootercmd = RPMLOWPORT;
					shooterCtrl->setDesiredValue(shootercmd);

					//Set the tilt down
					autotiltdown_sw = 1;

					//Set the hood down (high trajectory shot)
					autohooddown_sw = 1;

					//Set the intake deploy up
					autointakeretract_sw = 1;

					//Turn the shooter off
					autoshooteroff_sw = 1;

					//Turn off vision targeting
					targetingenabled = 0;

					//armcmd = ARMLOWPORT;
					//armCtrl->setDesiredValue(armcmd);
					//armCtrl->setConstants(ARMGAIN_P, ARMGAIN_I, ARMGAIN_D);
					//armCtrl->setErrorEpsilon(0.5);
					//armCtrl->setMinDoneCycles(0);
					//armCtrl->setMaxOutput(MAXARMSPEEDAUTO);

			   		//LatchSmartValues(); 

					//armlatch = 1;

					sblowportactive = 1;

					autolowport_sw = 0;

					//lowport_LED = 1;
		 		}
		 
			}
			else if ( (wallshot_sw == 1) || (cwall_sw == 1) || (autowallshot_sw == 1) )	//Close wall shot position
			{

				if (sbwallshotactive == 0)
				{

					SmartReset();  //Reset previous smart function and record last smart function state

					shootercmd = RPMWALLSHOT;
					shooterCtrl->setDesiredValue(shootercmd);

					//Set the tilt up
					autotiltup_sw = 1;

					//Set the hood down (high trajectory shot)
					autohooddown_sw = 1;

					//Set the intake deploy up
					autointakeretract_sw = 1;
					
					//Turn the shooter off
					autoshooteroff_sw = 1;

					//Turn off vision targeting
					targetingenabled = 0;

					//armcmd = ARMWALLSHOT;
					//armCtrl->setDesiredValue(armcmd);
				 	//armCtrl->setConstants(ARMGAIN_P, ARMGAIN_I, ARMGAIN_D);
					//armCtrl->setErrorEpsilon(1.0);
					//armCtrl->setMinDoneCycles(0);
					//armCtrl->setMaxOutput(MAXARMSPEEDAUTO);

			   		//LatchSmartValues();

					//armlatch = 1;

					sbwallshotactive = 1;

					autowallshot_sw = 0;

					//wallshot_LED = 1;
		 		}

			}
			else if ( (fronttrench_sw == 1) || (ctrenchfront_sw == 1) || (autofronttrench_sw == 1) )	 //Front of trench shot position
			{	
					
				if (sbfronttrenchactive == 0)
				{
					SmartReset();  //Reset previous smart function and record last smart function state

					shootercmd = RPMFRONTTRENCH;
					shooterCtrl->setDesiredValue(shootercmd);

					//Set the tilt up
					autotiltup_sw = 1;

					//Set the hood up (regular shot)
					autohoodup_sw = 1;

					//Set the intake deploy up
					autointakeretract_sw = 1;
					
					//Turn the shooter off
					autoshooteroff_sw = 1;

					//Turn on vision targeting
					targetingenabled = 1;

					//armcmd = ARMFRONTTRENCH - 2.0;  //BLAR NEW STUFF - FORCE ARM TO STICK TO VELCRO
					//armCtrl->setDesiredValue(armcmd);
					//armCtrl->setConstants(ARMGAIN_P, ARMGAIN_I, ARMGAIN_D);
					//armCtrl->setErrorEpsilon(1.0);
					//armCtrl->setMinDoneCycles(0);
					//armCtrl->setMaxOutput(MAXARMSPEEDAUTO);

					//LatchSmartValues();

					//armlatch = 1;

					sbfronttrenchactive = 1;

					autofronttrench_sw = 0;

					//fronttrench_LED = 1;
		 		}
		 
			}
			else if ( (throughtrench_sw == 1) || (cthroughtrench_sw == 1) || (autothroughtrench_sw == 1) )		//Setup driving under control panel
			{
	
				if (sbthroughtrenchactive == 0)
				{
					SmartReset();  //Reset previous smart function and record last smart function state

					shootercmd = RPMTHROUGHTRENCH;
					shooterCtrl->setDesiredValue(shootercmd);

					//Set the tilt down
					autotiltdown_sw = 1;

					//Set the hood down (high trajectory shot) 
					autohooddown_sw = 1;

					//Set the intake deploy up
					autointakeretract_sw = 1;

					//Turn the shooter off
					autoshooteroff_sw = 1;

					//Turn off vision targeting
					targetingenabled = 0;

					//armcmd = ARMTHROUGHTRENCH;
					//armCtrl->setDesiredValue(armcmd);
					//armCtrl->setConstants(ARMGAIN_P, ARMGAIN_I, ARMGAIN_D);
					//armCtrl->setErrorEpsilon(0.5);
					//armCtrl->setMinDoneCycles(0);
					//armCtrl->setMaxOutput(MAXARMSPEEDAUTO);

			   		//LatchSmartValues(); 

					//armlatch = 1;

					sbthroughtrenchactive = 1;

					autothroughtrench_sw = 0;

					//throughtrench__LED = 1;
		 		}
		 
			} 

		 	smartbuttonon = 1;
		 	smartbuttonlock = 1;	//This serves as a oneshot to drive onetime actions (state changes) in the code below
		 		
		} // end if smartbutton is pressed

		//
		//Employ motion-restrictions for safe arm operation
		//
		
		if (sbpassmodeactive == 1)
		{
			//printf("Pass Mode Active\n");

			if (smartbuttonlock == 1)
			{
	
				//
				
			}

		}
		if (sbhomeshotactive == 1)
		{
			//printf("Home Shot Active\n");

			if (smartbuttonlock == 1)
			{

				//

			}

		}
		if (sbfarshotactive == 1)
		{
			//printf("Far Shot Active\n");
			
			if (smartbuttonlock == 1)
			{		

				//
				
			}
			
		}
		if (sbclimbmodeactive == 1)
		{
			//printf("Climb Mode Active\n");
			
			if (smartbuttonlock == 1)
			{

				//			
				
			}
						
		}
		if (sbspareshotactive == 1)
		{
			//printf("Spare Shot Active\n");

			if (smartbuttonlock == 1)
			{

				//

			}
			
		}
		if (sblowportactive == 1)
		{
			//printf("Low Port Active\n");

			if (smartbuttonlock == 1)
			{

				//

			}
			
		}
		if (sbwallshotactive == 1)
		{
			//printf("Wall Shot Active\n");

			if (smartbuttonlock == 1)
			{

				//

			}
			
		}
		if (sbfronttrenchactive == 1)
		{
			//printf("Front Trench Active\n");

			if (smartbuttonlock == 1)
			{

				//

			}
			
		}
		if (sbthroughtrenchactive == 1)
		{
			//printf("Through Trench Active\n");
			
			if (smartbuttonlock == 1)
			{

				//
				
			}
			
		}

		//Cancel Smartbutton lock if the execute pushbutton has been released
		
		if (smartbuttonlock == 1)
		{
			//if (sbexecute_sw == 0) smartbuttonlock = 0;	//Reset the oneshot after the execute smartbutton is released
		
			if ( ( (autonactive == 1) && (autodriveenable == 1) ) || (sbexecute_sw == 0) ) smartbuttonlock = 0;	//Reset the oneshot after the execute smartbutton is released
		}
		
	} //end if bigred_sw == 1
	
} // end DoSmartStuff


void XM23::BigRedBash (void)
{

	
	//Reset all Smart Button latches and Virtual Smart Buttons

	if (smartbuttonon == 1)
	{

		//lastsmartstate = SMARTOFF;

		//bigredrelease = 1;
			
		SmartReset();	//Reset All Smart Buttons

	}

}

void XM23::SmartReset (void)
{
	
    if (sbpassmodeactive == 1) lastsmartstate = PASSMODE;
    if (sbhomeshotactive == 1) lastsmartstate = HOMESHOT;
    if (sbfarshotactive == 1) lastsmartstate = FARSHOT;
    if (sbclimbmodeactive == 1) lastsmartstate = CLIMBMODE;
    if (sbspareshotactive == 1) lastsmartstate = SPARESHOT;
    if (sblowportactive == 1) lastsmartstate = LOWPORT;
    if (sbwallshotactive == 1) lastsmartstate = WALLSHOT;
    if (sbfronttrenchactive == 1) lastsmartstate = FRONTTRENCH;
	if (sbthroughtrenchactive == 1) lastsmartstate = THROUGHTRENCH;

	smartbuttonon = 0;
	smartbuttonlock = 0;
				
  	steerlatch = 0; 
  	//armlatch = 0;

  	//armcmd = armpos;
  	//adjustlock = 0;  	

  	autoshooteroff_sw = 1;
  	shooterstate = OFF;
  	shooterCtrl->setConstants(0.0, 0.0, 0.0);
	shootercmd = 0;
  	
	//Reset all virtual control buttons
	ResetAutoVars();

	//armCtrl->setConstants(0.0, 0.0, 0.0);

	passmode_sw = 0;
	homeshot_sw = 0;
	farshot_sw = 0;
	climbmode_sw = 0;
	spareshot_sw = 0;
	lowport_sw = 0;
	wallshot_sw = 0;
	fronttrench_sw = 0;
	throughtrench_sw = 0;

	autopassmode_sw = 0;
	autohomeshot_sw = 0;
	autofarshot_sw = 0;
	autoclimbmode_sw = 0;
	autospareshot_sw = 0;
	autolowport_sw = 0;
	autowallshot_sw = 0;
	autofronttrench_sw = 0;
	autothroughtrench_sw = 0;

    sbpassmodeactive = 0;
	sbhomeshotactive = 0;
    sbfarshotactive = 0;
    sbclimbmodeactive = 0;
	sbspareshotactive = 0;
    sblowportactive = 0;
    sbwallshotactive = 0;
    sbfronttrenchactive = 0;
	sbthroughtrenchactive = 0;

	
}
 
void XM23::BlinkControl(void)	//Master Blinker Control for LED's
{


	if (blinktimer->Get() > BLINKONTIME)	//turn the blinker on
	{
		blinkon = ! blinkon;	//Change the state of the boolean

		blinktimer->Reset(); 	//Reset the timer
	}
	else
	{
		//do nothing - waiting for timeout;
	}

	
	//if (blinkon == true)
	//{
	//	printf("Blink on!\n");
	//}
	//else
	//{
	//	printf("Blink off!\n");			
	//}
	

}


void XM23::CustomDashboard(void)
{

	// Get the current going through PDP channels, in Amperes.
	// The PDP returns the current in increments of 0.125A.
	// At low currents the current readings tend to be less accurate.

	//48
	
	// Get the current going through PDP channels, in Amperes.
	// The PDP returns the current in increments of 0.125A.
	// At low currents the current readings tend to be less accurate.
	frc::SmartDashboard::PutNumber("LMID Drive Current", m_pdp->GetCurrent(0));		//WHT     - OK
	frc::SmartDashboard::PutNumber("LF Drive Current", m_pdp->GetCurrent(1));       //BLU     - OK
	frc::SmartDashboard::PutNumber("Shooter Top Current", m_pdp->GetCurrent(2));	//CAN1    - OK
	frc::SmartDashboard::PutNumber("Shooter Bottom Current", m_pdp->GetCurrent(3));	//CAN2    - OK
	frc::SmartDashboard::PutNumber("Climber Current", m_pdp->GetCurrent(4));		//CAN3 	  - OK
	frc::SmartDashboard::PutNumber("Conveyor Feed Current", m_pdp->GetCurrent(5));  //CAN10   - OK       	
	frc::SmartDashboard::PutNumber("Intake Current", m_pdp->GetCurrent(6));			//GRN     - OK
	frc::SmartDashboard::PutNumber("R2-D2 Current", m_pdp->GetCurrent(7));			//CAN11   - OK
	frc::SmartDashboard::PutNumber("Blinkin LED Current", m_pdp->GetCurrent(8));	//YEL     - OK
	frc::SmartDashboard::PutNumber("12VDC Sensor Current", m_pdp->GetCurrent(9));	//12VDC   - OK
	//frc::SmartDashboard::PutNumber("Spare Current", m_pdp->GetCurrent(10));
	//frc::SmartDashboard::PutNumber("Spare Current", m_pdp->GetCurrent(11));			
	frc::SmartDashboard::PutNumber("RR Drive Current", m_pdp->GetCurrent(12));		//ORA     - OK
	frc::SmartDashboard::PutNumber("RMID Drive Current", m_pdp->GetCurrent(13));	//GRY     - OK
	frc::SmartDashboard::PutNumber("RF Drive Current", m_pdp->GetCurrent(14));	    //RED     - OK
	frc::SmartDashboard::PutNumber("LR Drive Current", m_pdp->GetCurrent(15));		//PUR     - OK

	//Get the total current being drawn by all monitored channels, in Amps.
	//frc::SmartDashboard::PutNumber("Total Current", m_pdp->GetTotalCurrent());
	// Get the voltage going into the PDP, in Volts.
	// The PDP returns the voltage in increments of 0.05 Volts.
	frc::SmartDashboard::PutNumber("Battery Voltage", m_pdp->GetVoltage());
	// Retrieves the temperature of the PDP, in degrees Celsius.
	//frc::SmartDashboard::PutNumber("Temperature", m_pdp->GetTemperature());

	//frc::SmartDashboard::PutNumber("Arm Position", armpos);
	//frc::SmartDashboard::PutNumber("Arm Speed", armspeed);
	//frc::SmartDashboard::PutNumber("Arm Command", armcmd);
	//frc::SmartDashboard::PutBoolean("Arm In Position", armposok);

	//frc::SmartDashboard::PutBoolean("Vision Lock", TV);
	//frc::SmartDashboard::PutNumber("Vision X",TX);
	//frc::SmartDashboard::PutNumber("Vision Y",TY);
	//frc::SmartDashboard::PutNumber("Vision Area",TArea);
	//frc::SmartDashboard::PutNumber("Vision Skew",TSkew);

	frc::SmartDashboard::PutNumber("R2-D2 Control Panel Degrees", r2d2pos);
	frc::SmartDashboard::PutNumber("Ball Feed Distance", ballfeedpos);

	frc::SmartDashboard::PutNumber("Climb Encoder", climbpos);
	frc::SmartDashboard::PutNumber("Climb Speed", climbspeed);
	frc::SmartDashboard::PutNumber("Climb Command", climbcmd);
	frc::SmartDashboard::PutBoolean("Climber In Position", climbposok);
	frc::SmartDashboard::PutBoolean("Climb Latch", climblatch);

	frc::SmartDashboard::PutBoolean("Climb Ratchet Lock ON", climblockon);

	frc::SmartDashboard::PutNumber("MAX SPEED:", (int) (maxspeed * 100.0) );

	frc::SmartDashboard::PutNumber("Auton Step", autonStep);
	
	frc::SmartDashboard::PutNumber("Gyro Angle", gyroangle);
	frc::SmartDashboard::PutNumber("Right Encoder", distanceright);
	frc::SmartDashboard::PutNumber("Steer Correct", steercorrect);
    //frc::SmartDashboard::PutNumber("Left Drive Output Command", LFdrive.speed);
	frc::SmartDashboard::PutNumber("Left Y", leftycalc);
	frc::SmartDashboard::PutNumber("Right Y", rightycalc);
	frc::SmartDashboard::PutNumber("Copilot Right Joy Y", copadrightjoyY);

	frc::SmartDashboard::PutNumber("Shooter Command: ", shootercmd);
	frc::SmartDashboard::PutNumber("Shooter Speed: ", shooterRPM);
	frc::SmartDashboard::PutBoolean("Shooter Ready", speedok);	
	frc::SmartDashboard::PutBoolean("Target Detected", targetdetected);
	frc::SmartDashboard::PutBoolean("Target Locked", targetlocked);
	frc::SmartDashboard::PutBoolean("OK to Shoot", oktoshoot);

	frc::SmartDashboard::PutBoolean("High Gear", highgearon);
	frc::SmartDashboard::PutBoolean("Low Gear", lowgearon);

	frc::SmartDashboard::PutBoolean("Ball Present Low", ballpresentlow);
	frc::SmartDashboard::PutBoolean("Ball Present High", ballpresenthigh);
	
	//frc::SmartDashboard::PutBoolean("Arm At Upper", armatupper);
	//frc::SmartDashboard::PutBoolean("Arm At Lower", armatlower);

	frc::SmartDashboard::PutBoolean("Climber Extend Limit", climbextended);
	frc::SmartDashboard::PutBoolean("Climber Retract Limit", climbretracted);

	frc::SmartDashboard::PutNumber("LED Code", blinkincode);

	//String Writes to replace Write2LCD
	/*

	if (pickright == PICKRIGHT)
	{
		frc::SmartDashboard::PutString("SELECT TEAM: ", "GO CAVS!");
	}
	else
	{
		frc::SmartDashboard::PutString("SELECT TEAM: ", "GO BROWNS!");
	}

	

	if (switchstraightstate == RUNSWITCH)
	{
		frc::SmartDashboard::PutString("Choose Switch or Straight! ", "SWITCH CHOSEN!");
	}
	else
	{
		frc::SmartDashboard::PutString("Choose Switch or Straight! ", "STRAIGHT CHOSEN!");
	}

	*/

	/*
	if (shiftstate == SHIFTHIGH)
	{
		frc::SmartDashboard::PutString("GEAR STATUS: ", "HIGH GEAR");
	}
	else
	{
		frc::SmartDashboard::PutString("GEAR STATUS: ", "LOW GEAR");
	}
	*/

	if (controlstate == JOYSTICK)
	{
		frc::SmartDashboard::PutString("PILOT CONTROL: ", "JOYSTICK");
	}
	else //controlstate == GAMEPAD
	{
		frc::SmartDashboard::PutString("PILOT CONTROL: ", "GAMEPAD");
	}

	if (autodrivestate == DRIVENORMALAUTO)
	{
		frc::SmartDashboard::PutString("AUTO DRIVE MODE: ", "GYROASSIST ACTIVE");
	}
	else
	{
		frc::SmartDashboard::PutString("AUTO DRIVE MODE: ", "GYROASSIST DISABLED");
	}

	
	if (shootermode == AUTOSHOOT)
	{
		frc::SmartDashboard::PutString("Shooter Mode: ", "AUTOMATIC");
	}
	else
	{
		frc::SmartDashboard::PutString("Shooter Mode: ", "MANUAL");
	}
	

	//Only update these dashboard values when the robot is disabled
	if (frc::DriverStation::GetInstance().IsDisabled())
	{

		frc::SmartDashboard::PutBoolean("Blue Alliance", allianceisblue);
		frc::SmartDashboard::PutBoolean("Red Alliance", allianceisred);
		frc::SmartDashboard::PutNumber("AUTO DELAY IN SEC: ", delayset);
		frc::SmartDashboard::PutString("Message of the Day: ", "Make it ugly!");

		
		
		if (drivestate == ARCADEMODE)
		{
			frc::SmartDashboard::PutString("CONTROL MODE: ", "1-STICK ARCADE");
		}
		else
		{
			frc::SmartDashboard::PutString("CONTROL MODE: ", "2-STICK TANK");
		}

		/*
		AUTONOMOUS ROUTINES SUMMARY

			2020 Programs - Left Alliance Starting Position
			Left 1 - Full Manual Control
			Left 2 - Default - Pick up 2 Balls from Opponent Trench, Shoot 5
			Left 3 - 
			Left 4 - 
			Left 5 - 
			Left 6 - 
			Left 7 - Drive Forward Only
			Left 8 - Do Nothing

			2020 Programs - Center Alliance Starting Position
			Center 1 - Full Manual Control
			Center 2 - Default - Shoot 3 Balls, Move off Initiation Line
			Center 3 - 
			Center 4 - 2021 AUTONAV CHALLENGE - LOW GEAR BOUNCE PATH (BALLS)
			Center 5 - 2021 AUTONAV CHALLENGE - LOW GEAR SLALOM PATH (FISH)
			Center 6 - 
			Center 7 - Drive Forward Only
			Center 8 - Do Nothing

			2020 Programs - Right Alliance Starting Position
			Right 1 - Full Manual Control
			Right 2 - Default - Shoot 3 Balls, Gather 3 from Trench, Shoot 3
			Right 3 - 
			Right 4 - 2021 AUTONAV CHALLENGE - HIGH GEAR BOUNCE PATH (BALLS)
			Right 5 - 2021 AUTONAV CHALLENGE - HIGH GEAR SLALOM PATH (FISH)
			Right 6 - 2021 AUTONAV CHALLENGE - HIGH GEAR BARREL RACING PATH
			Right 7 - Drive Forward Only
			Right 8 - Do Nothing

		*/

		switch (autoprognum)
		{
			case 1: //Program #1
				switch (autostartpos)
				{
					case 1:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 LEFT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "1: FULL MANUAL CONTROL");
						break;
					case 2:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 CENTER ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "1: FULL MANUAL CONTROL");
						break;
					case 3:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 RIGHT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "1: FULL MANUAL CONTROL");
						break;
				}
				break;
			case 2: //Program #2
				switch (autostartpos)
				{
					case 1:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 LEFT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "2: PICK UP 2, SHOOT 5");
						break;
					case 2:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 CENTER ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "2: SHOOT 3 AND MOVE OFF LINE");
						break;
					case 3:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 RIGHT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "2: SHOOT 3, PICK UP & SHOOT 3");
						break;
				}
				break;
			case 3: //Program #3
				switch (autostartpos)
				{
					case 1:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 LEFT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "3: SPARE");
						break;
					case 2:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 CENTER ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "3: SPARE");
						break;
					case 3:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 RIGHT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "3: SPARE");
						break;
				}
				break;
			case 4: //Program #4
				switch (autostartpos)
				{
					case 1:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 LEFT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "4: SPARE");
						break;
					case 2:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 CENTER ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "4: 2021 AUTONAV BOUNCE LOW GEAR");
						break;
					case 3:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 RIGHT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "4: 2021 AUTONAV BOUNCE HIGH GEAR");
						break;
				}
				break;
			case 5: //Program #5
				switch (autostartpos)
				{
					case 1:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 LEFT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "5: SPARE");
						break;
					case 2:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 CENTER ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "5: AUTONAV SLALOM LOW GEAR");
						break;
					case 3:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 RIGHT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "5: AUTONAV SLALOM HIGH GEAR");
						break;
				}
				break;
			case 6: //Program #6
				switch (autostartpos)
				{
					case 1:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 LEFT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "6: SPARE");
						break;
					case 2:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 CENTER ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "6: SPARE");
						break;
					case 3:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 RIGHT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "6: AUTONAV BARREL RACING HIGH GEAR");
						break;
				}
				break;
			case 7: //Program #7
				switch (autostartpos)
				{
					case 1:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 LEFT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "7: DRIVE BACKWARD ONLY");
						break;
					case 2:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 CENTER ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "7: DRIVE BACKWARD ONLY");
						break;
					case 3:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 RIGHT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "7: DRIVE BACKWARD ONLY");
						break;
				}
				break;
			case 8: //Program #8
				switch (autostartpos)
				{
					case 1:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 LEFT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "8: DO NOTHING");
						break;
					case 2:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 CENTER ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "8: DO NOTHING");
						break;
					case 3:
						frc::SmartDashboard::PutString("AUTON PROGRAM TYPE: ", "2020 RIGHT ALLIANCE MODES");
						frc::SmartDashboard::PutString("PROGRAM INFO: ", "8: DO NOTHING");
						break;
				}
				break;

		}

	}

}


void XM23::ResetAutoVars(void)
{
	autotiltdown_sw = 0;
	autotiltup_sw = 0;
	autohoodup_sw = 0;
	autohooddown_sw = 0;
	autointakedeploy_sw = 0;
	autointakeretract_sw = 0;
	autoclimblockon_sw = 0;
	autoclimblockoff_sw = 0;

	autosuck_sw = 0;
	autospit_sw = 0;
	autoballfeedload_sw = 0;
	autoballfeedeject_sw = 0;
	autoballfeedshoot_sw = 0;
	autoshooteron_sw = 0;
  	autoshooteroff_sw = 0;
	autoclimbretract_sw = 0;
	autoclimbextend_sw = 0;
	autor2d2manual_sw = 0;
	autor2d2color_sw = 0;
	
	autopadrightjoyY = 0.0;


}


void XM23::DoLEDStuff(void)
{
	
	//****************************
	//Blinkin LED Module Control 
	//****************************

	//Default alliance coloring

	#define BLINKINDWELL	2.5  //leave the latest blink code on for this duration

	if (blinkintimer->Get() > BLINKINDWELL)	//Revert to the default alliance color pattern after timer times out
	{

		if (alliancestate == BLUEALLIANCE)
		{
			allianceisblue = 1;
			blinkincode = BLUESOLID;
		}
		else if (alliancestate == REDALLIANCE) 
		{
			allianceisblue = 0;
			blinkincode = REDSOLID;
		}

	}

	if (blinkinoverride_sw == 1)
	{
		blinkincode = speedadjust;
	}
	else
	{
		//
	}

	blinkin->Set(blinkincode);

  


}

 void XM23::GetAnalogSensors(void)
 {

	 	ballvallow = balllowsens->GetAverageVoltage();
	
		if (ballvallow > AUTOMATIONDIRECTPRESENTTHRESH)
		{
			ballpresentlow = 1;
		}
		else
		{
			ballpresentlow = 0;
		}



		ballvalhigh = balllowsens->GetAverageVoltage();
	
		if (ballvalhigh > AUTOMATIONDIRECTPRESENTTHRESH)
		{
			ballpresenthigh = 1;
		}
		else
		{
			ballpresenthigh = 0;
		}


}

void XM23::CANInit(void)
{

	/* CAN ID's
	ID 0 - PDP, PCM

	ID 1 - Upper Shooter Falcon 500		(shootertop)
	ID 2 - Lower Shooter Falcon 500		(shooterbottom)
	ID 3 - Climber Falcon 500			(climber)
	ID 10 - Ball Feed Conveyor			(ballfeed)
	ID 11 - R2-D2 NEO 550				(r2d2)

	*/

	//*****************************************
	//Configure VEX/Phoenix CAN Devices
	//*****************************************	

	//Config any motor inversions and follower motors for paired-motor mechanisms

	shootertop->SetInverted(false);
	shooterbottom->Follow(*shootertop);
	shooterbottom->SetInverted(InvertType::FollowMaster);
	
	climber->SetInverted(false);

	//Config and enable voltage compensation
	shootertop->ConfigVoltageCompSaturation(11);
	//ENABLE THIS?  2021 - shooterbottom->ConfigVoltageCompSaturation(11);
	climber->ConfigVoltageCompSaturation(11);
	shootertop->EnableVoltageCompensation(true);
	//ENABLE THIS?  2021 - shooterbottom->EnableVoltageCompensation(true);
	climber->EnableVoltageCompensation(true);

	//Reset all Phoenix CAN devices to factory default upon startup
	shootertop->ConfigFactoryDefault(kTimeoutMs);
	shooterbottom->ConfigFactoryDefault(kTimeoutMs);
	climber->ConfigFactoryDefault(kTimeoutMs);

	//Choose the sensor associated with the CAN device
	shootertop->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
	climber->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);

	//Set brake/coast modes
	//BLAR 2021
	shootertop->SetNeutralMode(NeutralMode::Brake);
	shooterbottom->SetNeutralMode(NeutralMode::Brake);

	climber->SetNeutralMode(NeutralMode::Brake);

	//Set stator current limits
	shootertop->ConfigSupplyCurrentLimit(talonCurrentLimitShooter, kTimeoutMs);
	shooterbottom->ConfigSupplyCurrentLimit(talonCurrentLimitShooter, kTimeoutMs);
	climber->ConfigSupplyCurrentLimit(talonCurrentLimitClimber, kTimeoutMs);

	//Set the peak and nominal outputs
	shootertop->ConfigNominalOutputForward(0, kTimeoutMs);
	shootertop->ConfigNominalOutputReverse(0, kTimeoutMs);
	shootertop->ConfigPeakOutputForward(1, kTimeoutMs);
	shootertop->ConfigPeakOutputReverse(-1, kTimeoutMs);

	shooterbottom->ConfigNominalOutputForward(0, kTimeoutMs);
	shooterbottom->ConfigNominalOutputReverse(0, kTimeoutMs);
	shooterbottom->ConfigPeakOutputForward(1, kTimeoutMs);
	shooterbottom->ConfigPeakOutputReverse(-1, kTimeoutMs);

	climber->ConfigNominalOutputForward(0, kTimeoutMs);
	climber->ConfigNominalOutputReverse(0, kTimeoutMs);
	climber->ConfigPeakOutputForward(1, kTimeoutMs);
	climber->ConfigPeakOutputReverse(-1, kTimeoutMs);

	/* set closed loop gains in slot0 */
	shootertop->Config_kF(kPIDLoopIdx, 1.2488, kTimeoutMs);
	shootertop->Config_kP(kPIDLoopIdx, 1.4985, kTimeoutMs);
	shootertop->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
	shootertop->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

	//shooterbottom->Config_kF(kPIDLoopIdx, 1.2488, kTimeoutMs);
	//shooterbottom->Config_kP(kPIDLoopIdx, 1.4985, kTimeoutMs);
	//shooterbottom->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
	//shooterbottom->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

	//climber->Config_kF(kPIDLoopIdx, 1.2488, kTimeoutMs);
	//climber->Config_kP(kPIDLoopIdx, 1.4985, kTimeoutMs);
	//climber->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
	//climber->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);


	//*****************************************
	//Configure REV Robotics CAN Devices
	//*****************************************	
	
	//Reset all REV Robotics CAN devices to factory default upon startup
	ballfeed->RestoreFactoryDefaults();
	//r2d2->RestoreFactoryDefaults();

	//Choose the sensor associated with the CAN device

	//Set brake/coast modes
	ballfeed->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	//r2d2->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

	//Set supply current limits
	ballfeed->SetSmartCurrentLimit(30);
	//r2d2->SetSmartCurrentLimit(30);

}

void XM23::DoCANStuff(void)
{
	
	double motorOutput = shootertop->GetMotorOutputPercent();
	
	/* prepare line to print */
	_sb.append("\tout:");
	_sb.append(std::to_string(motorOutput));
	_sb.append("\tspd:");
	_sb.append(std::to_string(shootertop->GetSelectedSensorVelocity(kPIDLoopIdx)));
	
	
	/* while button1 is held down, closed-loop on target velocity */
	if (1 == 0) //drive_joystickL->GetRawButton(1)) 
	{
		/* Speed mode */
		/* Convert 60 RPM to units / 100ms.
			* 4096 Units/Rev * 60 RPM / 600 100ms/min in either direction:
			* velocity setpoint is in units/100ms
			*/
		double targetVelocity_UnitsPer100ms = leftjoyY * 60.0 * 4096 / 600;
		/* 60 RPM in either direction */
		shootertop->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms);

		/* append more signals to print when in speed mode. */
		_sb.append("\terrNative:");
		_sb.append(std::to_string(shootertop->GetClosedLoopError(kPIDLoopIdx)));
		_sb.append("\ttrg:");
		_sb.append(std::to_string(targetVelocity_UnitsPer100ms));
	} 
	else 
	{
		/* Percent voltage mode */
		
		//shootertop->Set(ControlMode::PercentOutput, leftjoyY);
		
		//shooterbottom->Set(ControlMode::PercentOutput, -leftjoyY);

	}

	/* print every ten loops, printing too much too fast is generally bad for performance */
	if ( (1 == 0) && (++_loops >= 10) ) 
	{
		_loops = 0;
		printf("%s\n",_sb.c_str());
	}

	_sb.clear();
	

}// DoCANStuff

void XM23::LightsOn(void)
{

		//Force Limelight LED's ON	
		nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",3);

}

void XM23::LightsOff(void)
{
	
		//Force Limelight LED's OFF
		nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",1);


}

void XM23::DoLimeStuff()
{

		//Limelight Network Tables - this is called once in RobotInit - reference only here
		std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

		//Set PIP Secondary Stream Mode
		nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("stream",2);

		TX = table->GetNumber("tx",0.0);
		TY = table->GetNumber("ty",0.0);
		TV = table->GetNumber("tv",0.0);
		TArea = table->GetNumber("ta",0.0);
		TSkew = table->GetNumber("ts",0.0);

		if ( (targetingenabled == 1) && (TV == 1) ) //A vision target has been detected
		{
			targetdetected = 1;

			blinkincode = COLOR1LARSON;		//YELLOW KITT LIGHT
			blinkintimer->Reset();

		}
		else
		{
			targetdetected = 0;
			targetlocked = 0;
		}

		if (targetdetected == 1)
		{
			if (abs(TX) < ONTARGETTHRESH)
			{
				targetlocked = 1;

				blinkincode = FIRELARGE;	//THROUGH THE FIRE AND THE FLAMES
				blinkintimer->Reset();
 
			}
			else
			{
				targetlocked = 0;
			}
			
		}
		

}
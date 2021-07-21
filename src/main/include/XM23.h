#ifndef XM23_H_
#define XM23_H_

#include <AugmentedEncoder.h>
#include <DoubleSolenoid48.h>
#include <LogitechGamepad.h>
#include <SimPID.h>
#include <XM23Lib.h>

#include "frc/WPILib.h"
#include "rev/SparkMax.h"
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"
#include "ctre/Phoenix.h"

#include "adi/ADIS16470_IMU.h"

#include "frc/Solenoid.h"
#include "cameraserver/CameraServer.h"
#include "cscore.h"
#include "networktables/NetworkTable.h"
#include "tables/ITable.h"
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/core.hpp>
#include "wpi/StringRef.h"
#include "wpi/ArrayRef.h"
#include <thread>
#include <string>
#include <chrono>


//#include "AHRS.h"

//#include "Dashboard.h"
//#include "InsightLT\InsightLT.h"

class XM23 : public frc::TimedRobot
{
public:
	XM23();
	virtual void RobotInit();
	virtual void DisabledInit();
	virtual void DisabledPeriodic();
	virtual void DisabledContinuous();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TeleopContinuous();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void AutonomousContinuous();
	
	void GetControllerIO();
	void CANInit();
	void DoCANStuff();
	void DoLimeStuff();
	void DoDriveStuff(float lyval, float lxval, float ryval, float rxval);
	void DoShiftStuff();
	void DoSmartStuff();
	void DoOtherStuff();
	void BigRedBash();
	void LatchSmartValues();
	//void DoArmStuff();
	void DoClimbStuff();
	void DoShooterStuff();
	void SmartReset();
	void BlinkControl();
	void DriveNeutral();
	void CustomDashboard();
	void ResetAutoVars();
	void DoCameraStuff();
	void DoLEDStuff();
	void GetAnalogSensors();
	void LightsOn();
	void LightsOff();

	void AutoLeft1();
	void AutoLeft2();
	void AutoLeft3();
	void AutoLeft4();
	void AutoLeft5();
	void AutoLeft6();
	void AutoLeft7();
	void AutoLeft8();
	
	void AutoCenter1();
	void AutoCenter2();
	void AutoCenter3();
	void AutoCenter4();
	void AutoCenter5();
	void AutoCenter6();
	void AutoCenter7();
	void AutoCenter8();
	
	void AutoRight1();
	void AutoRight2();
	void AutoRight3();
	void AutoRight4();
	void AutoRight5();
	void AutoRight6();
	void AutoRight7();
	void AutoRight8();
	

private:

	//CTRE Current Limit Configs
    SupplyCurrentLimitConfiguration talonCurrentLimitClimber;
    #define   C_ENABLE_CURRENT_LIMIT = true;
    #define   C_CONTINUOUS_CURRENT_LIMIT = 25; //amps
    #define   C_TRIGGER_THRESHOLD_LIMIT = 35; //amps
    #define   C_TRIGGER_THRESHOLD_TIME = 0.4; //sec

    SupplyCurrentLimitConfiguration talonCurrentLimitShooter;
    #define   S_ENABLE_CURRENT_LIMIT = true;
    #define   S_CONTINUOUS_CURRENT_LIMIT = 35; //amps
    #define   S_TRIGGER_THRESHOLD_LIMIT = 45; //amps
    #define   S_TRIGGER_THRESHOLD_TIME = 0.4; //sec

	//CameraVariables
	//cs::CvSink sink1;
	//cs::CvSink sink2;
	//cs::MjpegServer server;
	cs::UsbCamera lowcam;
	cs::UsbCamera highcam;

	//LiveWindow *lw = LiveWindow::GetInstance();
	//frc::SendableChooser<std::string> *chooser;
	//const std::string autoNameDefault = "Default";
	//const std::string autoNameCustom = "My Auto";
	//std::string autoSelected;

	//frc::DriverStation *m_ds;

	frc::PowerDistributionPanel *m_pdp;

	//AHRS *navX;

	uint32_t prior_packet_number;
	uint8_t packets_in_second;

	static constexpr float G = 9.806605; //meters per second squared
	
	//For drivetrain encoders
	static constexpr float ticks_per_rev = 360.0;
	
	//Drivetrain wheel linear distance per revolution
	static constexpr float distance_per_rev = 18.85; 	//4" Wheel = 12.57, 6" Wheel = 18.85, 8" Wheel = 25.13 inches per rev

	//For arm encoder
	//static constexpr float lticks_per_rev = 1024.0;	//CTRE Magnetic Encoder

	//Arm angle rotary distance per revolution
	//static constexpr float adistance_per_rev = 60.0;	//6:1 reduction from VP integrated encoder to arm sprocket = 360.0 / 6 = 60 degrees per rev
	
	//Climber linear distance per motor revolution
	static constexpr float cdistance_per_rev = 0.2094;  //30:1 reduction from TalonFX integrated encoder to spool -> 1 spool rev = 2.00" dia. * pi = 6.2832" linear distance per spool rev / 30 = 0.2094" per motor rev    

	//Conveyor feed linear distance per motor revolution
	static constexpr float fdistance_per_rev = 0.35437;  //18T HTD pulley = 1.13" dia. * pi = 3.5437" linear distance per pulley rev (ignoring slip) / 10 = 0.35437" per motor rev

	//R2-D2 rotary control panel degrees per motor revolution
	static constexpr float rdistance_per_rev = 6.75;  //control panel = 360 degrees / (32/3 diameter ratio) / 5:1 belt reduction = 6.75 degrees per motor rev

	//For shooter and climber encoders (TalonFX Integrated)
	static constexpr float tticks_per_rev = 2048.0;

	//For conveyor feed and R2-D2 encoders (REV Robotics)
	static constexpr float rticks_per_rev = 42.0;




	struct {
	//describes left and right drive trains
		float speed; //current speed
		float adjust; //how much to adjust current speed
	}LFdrive, LRdrive, LMIDdrive, RFdrive, RRdrive, RMIDdrive;
	

	typedef enum{
		ARCADEMODE, TANKMODE
	} DriveState;
	
	typedef enum{
		JOYSTICK, GAMEPAD
	} ControlState;

	typedef enum{
		DRIVENORMALAUTO, DRIVENORMALMAN
	} AutoDriveState;

	//PNEUMATIC TYPES
	
	typedef enum {
		SHIFTHIGH, SHIFTLOW
	} ShiftState;

	typedef enum{
		OPENGRIP, CLOSEGRIP
	} GripperState;

	typedef enum{
		TILTUP, TILTDOWN
	} TiltState;

	typedef enum{
		HOODUP, HOODDOWN
	} HoodState;

	typedef enum{
		DEPLOYINTAKE, RETRACTINTAKE
	} IntakeDeployState;

	typedef enum{
		CLIMBLOCKOFF, CLIMBLOCKON
	} ClimbLockState;

	//MOTOR TYPES

	typedef enum{
		SUCK, SPIT, HOLD, INTAKEOFF
	} IntakeState;
	
	typedef enum{
		BALLFEEDLOAD, BALLFEEDSHOOT, BALLFEEDEJECT, BALLFEEDOFF
	} BallFeedState;

	typedef enum{
		R2D2MANUAL, R2D2STAGE2, R2D2COLORPICK, R2D2OFF
	} R2D2State;

	typedef enum{
		CLIMBRETRACT, CLIMBEXTEND, CLIMBHOLD, CLIMBOFF
	} ClimbState;

	typedef enum{
		ON, OFF, REVERSE 
	} ShooterState;

	typedef enum{
		AUTOSHOOT, MANUALSHOOT
	} ShooterMode;

	typedef enum{
		JOYNEUTRAL, JOYUP, JOYDOWN, JOYLEFT, JOYRIGHT, JOY_UPLEFT, JOY_DOWNLEFT, JOY_DOWNRIGHT, JOY_UPRIGHT
	} StickState;

	typedef enum{
		PASSMODE, HOMESHOT, FARSHOT, CLIMBMODE, SPARESHOT, LOWPORT, WALLSHOT, FRONTTRENCH, THROUGHTRENCH, SMARTOFF
	} SmartState;

	//typedef enum{
	//	LEFTSCALE, RIGHTSCALE, SCALEERROR
	//} ScaleState;
	
	//typedef enum{
	//	LEFTSWITCH, RIGHTSWITCH, SWITCHERROR
	//} SwitchState;

	//typedef enum{
	//	RUNSWITCH, RUNSTRAIGHT
	//} SwitchStraightState;

	typedef enum{
		CP_RED, CP_YELLOW, CP_GREEN, CP_BLUE, CP_ERROR
	} ColorState;

	typedef enum{
			BLUEALLIANCE, REDALLIANCE 
	} AllianceState;

	uint32_t auto_periodic_loops;
	uint32_t disabled_periodic_loops;
	uint32_t teleop_periodic_loops;

	//Driver joystick
	frc::Joystick *drive_joystickL;
	frc::Joystick *drive_joystickR;

	//Operator gamepad
	LogitechGamepad *copilot_pad;
	LogitechGamepad *pilot_pad;


	//CAN motors - NEW FOR 2020

	TalonFX *shootertop;
	TalonFX *shooterbottom;
	TalonFX *climber;	
	rev::CANSparkMax *ballfeed; 	
	//rev::CANSparkMax *r2d2; 	


	//Drive motors 
	
	//48
	
	rev::SparkMax *driveRF;			//RED
	rev::SparkMax *driveRR;			//ORANGE
	rev::SparkMax *driveLR;			//PURPLE
	rev::SparkMax *driveLF;			//BLUE
	rev::SparkMax *driveRMID;		//GRAY
	rev::SparkMax *driveLMID;		//WHITE
	rev::SparkMax *intake;			//GREEN
	frc::VictorSP *blinkin;			//YELLOW
	//frc::VictorSP *spare;		    //BROWN
	//frc::VictorSP *spare;	    	//BLACK
	
	//OLD or TRANSFER for 2020
	//frc::VictorSP *spare;			//RED/WHITE
	//frc::VictorSP *spare;			//ORANGE/WHITE
	//frc::VictorSP *spare;			//WHITE/WHITE

	//Relays
	//frc::Relay *light;
	
	//Servos
	//frc::Servo *liftlock;
		
	//Air pump and pressure switch
	frc::Compressor *airpump;


	// Solenoid Valves
	
	//Low Pressure Manifold
	frc::Solenoid *shifter;
	
	//High Pressure Manifold
	frc::Solenoid *hood;
	DoubleSolenoid48 *tilt;
	DoubleSolenoid48 *intakedeploy;
	DoubleSolenoid48 *climblock;

	// Robot sensors

	frc::ADIS16470_IMU *gyro;
	//frc::ADXRS450_Gyro *gyro;

	
	//frc::AnalogGyro *gyro;
	frc::AnalogInput *balllowsens;
	frc::AnalogInput *ballhighsens;
	//frc::AnalogInput *sparesens;

	float ballvallow;
	float ballvalhigh;
	
	//AnalogInput *rangesens;
	
	//frc::DigitalInput *ballsensorlow;
	//frc::DigitalInput *ballsensorhigh;
	frc::DigitalInput *armdownlimit;
	frc::DigitalInput *armuplimit;
	frc::DigitalInput *climbretractlimit;
	frc::DigitalInput *climbextendlimit;


	//frc::DigitalOutput *pingoff;
	
	//AugmentedEncoder *ArmEncoder;
	//AugmentedEncoder *LeftEncoder;

	AugmentedEncoder *RightEncoder;
	//rev::CANEncoder *R2D2Encoder;
	rev::CANEncoder *BallFeedEncoder;

		
	// Robot PID controls
	SimPID *EncoderYCtrl;
	SimPID *GyroXCtrl;
	//SimPID *armCtrl;
	SimPID *climbCtrl;
	SimPID *shooterCtrl;

	frc::Timer *autotimer;
	frc::Timer *delaytimer;
	frc::Timer *shifttimer;
	frc::Timer *cargoholdtimer;
	frc::Timer *blinktimer;
	frc::Timer *blinkintimer;
	
	//SmartButton Variables
	//bool armlatch;			//Arm Smart Button Control Active
	bool climblatch;			//Climber Smart Button (PID) Control Active
	bool steerlatch;			//Gyro Steering Control Active

	//float  armcmd;			//Stores commanded arm angle position
	float  climbcmd;			//Stores commanded climber height position
	float  shootercmd;			//Stores commanded shooter wheel speed

	float autopadrightjoyY;    	//Virtual Manual Joystick

	bool oktoshoot;
	bool speedok;

	bool ballpresentlow;
	bool ballpresenthigh;

	//bool armatupper;
	//bool armatlower;
	//bool armposok;
	
	bool climbextended;				//limit switches
	bool climbretracted;
	bool climbposok;

	bool highgearon;
	bool lowgearon;

	bool slowlatch;
	
	//bool adjustlock;				//Permits the manual arm joystick to return to neutral before adjusting the smart command again
	bool sadjustlock;				//Permits the right copilot joystick to return to neutral before adjusting the smartspeed command again
	bool upadjustlock;				//Permits the right copilot joystick to return to neutral before permitting another upward movement near up travel limit
	
	bool climberadjustlock;   		//Permits the climber controls to return to neutral before adjusting the smart command again
	bool climberetractadjustlock; 	//Permits the climber controls to return to neutral before permitting another climber retract movement

	bool shiftlatch;

	bool autotiltup_sw;
	bool autotiltdown_sw;
	bool autohoodup_sw;
	bool autohooddown_sw;
	bool autointakedeploy_sw;
	bool autointakeretract_sw;
	bool autoclimblockon_sw;
	bool autoclimblockoff_sw;

	bool autosuck_sw;
	bool autospit_sw;
	bool autoballfeedload_sw;
	bool autoballfeedeject_sw;
	bool autoballfeedshoot_sw;
	bool autoclimbretract_sw;
	bool autoclimbextend_sw;
  	bool autoshooteron_sw;
  	bool autoshooteroff_sw;
	bool autor2d2manual_sw;
	bool autor2d2color_sw;

	bool ballholdlatch;

	bool autopassmode_sw;
	bool autohomeshot_sw;
	bool autofarshot_sw;
	bool autoclimbmode_sw;
	bool autospareshot_sw;
	bool autolowport_sw;
	bool autowallshot_sw;
	bool autofronttrench_sw;
	bool autothroughtrench_sw;
	
	bool passmode_sw;
	bool homeshot_sw;
	bool farshot_sw;
	bool climbmode_sw;
	bool spareshot_sw;
	bool lowport_sw;
	bool wallshot_sw;
	bool fronttrench_sw;
	bool throughtrench_sw;
	
	bool sbpassmodeactive;
	bool sbhomeshotactive;
	bool sbfarshotactive;
	bool sbclimbmodeactive;
	bool sbspareshotactive;
	bool sblowportactive;
	bool sbwallshotactive;
	bool sbfronttrenchactive;
	bool sbthroughtrenchactive;

	bool bigredrelease;
	bool smartbuttonon; 
	bool smartbuttonlock;
	
	//Field State Variables
	//SwitchState switchstate;
	//ScaleState scalestate;
	//SwitchStraightState switchstraightstate;
	
	ColorState colorstate;

	AllianceState alliancestate; //Stores the alliance color during the match
	bool blueallianceactive;

	SmartState lastsmartstate;	//Stores the last smart button state prior to current
	
	//Drivetrain/Shifting Variables
	AutoDriveState autodrivestate;				//Auto (Gyro-Assist) or Manual (Gyro-Assist disabled)
	DriveState drivestate;						//Arcade Mode (1-stick) or Tank Mode (2-stick)
	ControlState controlstate;					//Selects gamepad or joystick for pilot control
	ShiftState shiftstate;						//Stores state of shifting gearboxes
	StickState joyleftstate, joyrightstate;		//Stores current position of copilot gamepad joysticks
	
	float leftx;								//Used in DoDriveStuff
	float lefty;								//Used in DoDriveStuff
	float rightx;								//Used in DoDriveStuff
	float righty;								//Used in DoDriveStuff
	
	//DRIVETRAIN CONTROLLER IIR FILTERING
	float leftycalc;
	float leftxcalc;
	float rightycalc;
	float rightxcalc;
		
	float leftytau;
	float leftxtau;
	float rightytau;
	float rightxtau;
			
	float oldlefty;
	float oldleftx;
	float oldrighty;
	float oldrightx;
	
	//Scoring Mechanism Variables

	HoodState hoodstate;						//Stores current state of the shooter hood cylinder
	TiltState tiltstate;						//Stores current state of the shooter cage tilt cylinders
	IntakeDeployState intakedeploystate;		//Stores current state of the intake deployment cylinders
	ClimbLockState climblockstate;				//Stores current state of the climber lock cylinder

	IntakeState intakestate;					//Stores the current state of the power cell intake
	BallFeedState ballfeedstate;				//Stores the current state of the ball feed conveyor
	R2D2State r2d2state;						//Stores the current state of the R2D2 Color Wheel Manipulator
	ClimbState climbstate;						//Stores the current state of the climber

	ShooterState shooterstate;					//Stores current state of the shooter motors
	ShooterMode  shootermode;				    //Stores the current state of shooter operation - automatic (PID) or manual (Open loop)

	//float armspeed;							//Stores arm motor speed command 
	float ballfeedspeed;						//Stores ball feed drive speed command
	float intakespeed;							//Stores intake motor speed command
	float climbspeed;							//Stores climber motor speed command
	float r2d2speed;							//Stores r2d2 motor speed command
	
	float maxspeed;								//Stores maximum high gear driving voltage open loop command (speed)
	float maxspeeddisplay;						//Stores maximum high gear driving voltage as a percentage for display

	float blinkincode;							//Stores blinkin LED pattern command

	float gyrojoy;								//Stores gyro x-axis drive joystick command for drive straight code (PID)
	float encoderjoy;							//Stores encoder y-axis drive joystick command for drive straight code (PID)
	//float xvirt;								//Virtual Joystick Variables for Auton and Teleop Closed Loop Driving
	//float yvirt;
	
	float steercorrect;							//Teleop drive straight correction using gyro

	//int rangepos;								//Stores the ultrasonic sensor distance reading
		
	//Autonomous Variables
	
	int autonactive;
	int autodriveenable;						//2020 - enable to permit standard autonomous calculation
	
	int autonStep;
	int gyrodonestore;
	int encoderdonestore;
	
	int autostartpos;   //Autonomous starting position: 1 = Near, 3 = Far
	int autoprognum;    //Autonomous program number for starting position: 1-8 (or more)
	
	float delayset;     //Autonomous time delay variable set by right joystick throttle during disabled mode
	float speedset;		//speed adjustment variable set by left joystick throttle
	
	int autostartlock;
	int autoproglock;

	//Drive Encoder Variables
	float distanceleft;		//Left Encoder Count Storage
	float distanceright;	//Right Encoder Count Storage

	//Drive Gyro variables
	float gyroangle;
	float lastgyroangle;
	float gyrorate;

	//Shooter Encoder Variables
	float shooterspeed;
	float shooterRPM;	//Shooter RPM

	//Arm Encoder Variables
	//float armpos;		//Stores the arm angle reading in degrees, using quadrature encoder
	//float armoffset;	//Stores the arm offset angle to add to the raw encoder reading
	//bool  armishome;	//Stores whether the arm was ever homed at startup

	float climbpos;		//Stores the climber position reading in inches, using quadrature encoder
	float climboffset;   //Stores the climber offset distance to add to the raw encoder reading
	bool  climbishome;   //Stores whether the climber was ever homed at startup
	bool  climberenabled;  //Stores whether the climber is unlocked by the correct button combo

	float r2d2pos;		//Stores the R2-D2 control panel manipulator reading in CP revs
	
	float ballfeedpos;  	//Stores the ball feed conveyor position in inches
	float ballfeedoffset;	//Stores the ball feed conveyor offset distance to add to the raw encoder reading

	//CAN Debug Variables

	std::string _sb;
	int _loops = 0;


	//Joystick and Gamepad Variables
	
	//Left Joystick USB1 - Pilot
	float  leftjoyY;			//GetY();
	float  leftjoyX;          	//GetX();
	float  speedadjust;		   	//GetThrottle();
	bool   psuck_sw;			//GetTrigger();
	bool   r2d2manual_sw; 		//GetTop();
	bool   r2d2color_sw;    	//GetRawButton(3);
	bool   shiftlow_sw; 		//GetRawButton(4);
	bool   shifthigh_sw; 		//GetRawButton(5);
	bool   slowmode_sw;			//GetRawButton(6);
	bool   blinkinoverride_sw;	//GetRawButton(7);
	bool   joystickselect_sw;	//GetRawButton(8);
	bool   gamepadselect_sw;	//GetRawButton(9);
	bool   autodrive_sw; 		//GetRawButton(10);
	bool   manualdrive_sw; 		//GetRawButton(11);

	//Right Joystick USB2 - Pilot
	float  rightjoyY;		    //GetY();
	float  rightjoyX;         	//GetX();
	float  delayadjust;			//GetThrottle();
	bool   pspit_sw;			//GetTrigger();
	bool   intakeretract_sw;	//GetTop();
	bool   intakedeploy_sw;		//GetRawButton(3);	
	bool   ptiltdown_sw;		//GetRawButton(4);
	bool   ptiltup_sw;			//GetRawButton(5);
	bool   climbersafety_sw;	//GetRawButton(6);
	//bool   spare_sw;	 		//GetRawButton(7);
	bool   arcade_sw; 			//GetRawButton(8);
	bool   tank_sw;	 			//GetRawButton(9);
	bool   climberetract_sw;	//GetRawButton(10);
	bool   climberextend_sw;	//GetRawButton(11);
	
	//Logitech Gamepad USB3 - Copilot
	float copadleftjoyX;		//GetLeftX();
	float copadleftjoyY;		//GetLeftY();
	float copadrightjoyX;		//GetRightX();
	float copadrightjoyY;		//GetRightY();
	bool  ctrenchfront_sw;		//GetNumberedButton(1);
	bool  cfarshot_sw;			//GetNumberedButton(2);
	bool  cballfeedshoot_sw;	//GetNumberedButton(3);
	bool  cspareshot_sw;    	//GetNumberedButton(4);
	bool  cballfeedload_sw;		//GetNumberedButton(5);
	bool  shooteron_sw;			//GetNumberedButton(6);
	bool  cballfeedeject_sw;	//GetNumberedButton(7);
	bool  shooteroff_sw;		//GetNumberedButton(8);
	bool  cthroughtrench_sw;	//GetNumberedButton(9);
	bool  cwall_sw; 		   	//GetNumberedButton(10);
	bool  bigred_sw;			//GetLeftPush();
	bool  sbexecute_sw;	 		//GetRightPush();
	int   copilotPOV;			//GetPOV();

	//Logitech Gamepad USB4 - Pilot
	float padleftjoyX;			//GetLeftX();
	float padleftjoyY;			//GetLeftY();
	float padrightjoyX;			//GetRightX();
	float padrightjoyY;			//GetRightY();
	//bool  spare_sw;			//GetNumberedButton(1);
	//bool  spare_sw;  			//GetNumberedButton(2);
	//bool  spare_sw;		    //GetNumberedButton(3);
	//bool  spare_sw;		   	//GetNumberedButton(4);
	bool  padshiftlow_sw; 		//GetNumberedButton(5);
	bool  padshifthigh_sw;		//GetNumberedButton(6);
	bool  padarmdown_sw;		//GetNumberedButton(7);
	bool  padslowmode_sw;		//GetNumberedButton(8);
	//bool  spare_sw;			//GetNumberedButton(9);
	//bool  spare_sw;			//GetNumberedButton(10);
	bool  autopos_sw;			//GetLeftPush();
	bool  autoprog_sw; 			//GetRightPush();
	int   pilotPOV;				//GetPOV();

	//Miscellaneous Variables

	bool blinkon;  //Blink Timer Variable

	bool ledlatch;

	bool allianceisblue;
	bool allianceisred;

	bool climblocklatch;
	bool climblockon;

	//Limelight Variables
	double TX;
	double TY;
	double TV;
	double TArea;
	double TSkew;	

	bool targetdetected;
	bool targetlocked;
	bool targetingenabled;
	

	//NetworkTable table;

	
};

//START_ROBOT_CLASS(XM23);

#endif // XM23_H_

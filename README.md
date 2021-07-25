# About

Robot code for the 2021 FIRST Robotics Competition - Infinite Recharge

# Getting Started

1. **Complete** [Workstation Setup](https://github.com/teamelite48/workstation-setup)
1. **Clone** this repository to your local machine
1. **Open** XM23 project folder in 2021 WPILIb VS Code
1. **Click** "No, Don't ask again" button when prompted to import project into 2021
1. **Click** "Yes" button when asked to import Java project
1. **Click** on the WPI icon in the corner to open the WPI Command Pallet
1. **Select** Build Robot Code
1. **Select** cpp
1. **Observe** build was successful

# Notes

* Most of the code is contained in XM23.cpp/.h
* Look in the XM23() constructor for hardware defines (speed controllers, sensors, etc.) and initializing variables and default hardware states (setting motor speeds to off, etc.).
* The code in RobotInit() is run once during robot power on.
* After this, DisabledInit() runs once, then DisabledPeriodic() for as long as the robot state is disabled (including disabling the bot after running in Teleop or Autonomous Mode). 
* The robot state is controlled by either the FMS (Field Management System) when connected to an official field, or the Driver Station (the app running on the laptop with joysticks and gamepads connected to it) when running locally.
* Whenever you enable the robot in Teleoperated (human-controlled mode), TeleopInit() runs once, and then TeleopPeriodic(), which has LOTS of code inside it.
* Whenever you enable the robot in Autonomous mode, AutonomousInit() runs once, and then AutonomousPeriodic(), which has LOTS of code inside it.
You will notice that teleop and autonomous both use a lot of common subroutines. 

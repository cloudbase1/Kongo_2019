/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */


public class RobotMap {
	/* The following parameters are used to control various aspects of the robot. */
    /* In the past the students tended to put these parameters in various files  like */
	/* the drive train. In the future all tunable parameters should be located here for*/
	/* easy control. */

	// Used by drive train to limit the max speed of the motors. 
	public static final double  MAX_MOTOR_POWER = 0.50;
	
	//Joystick Ports
	public static final int LEFT_JOYSTICK_PORT = 0;
	public static final int RIGHT_JOYSTICK_PORT = 1;
	
	//Buttons

	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;
	
	//Left Motor Ports Port side motor
	public static final int LEFT_MOTOR_PORT = 7; 
	
	//Right Motor Ports Starboard side motor
	public static final int RIGHT_MOTOR_PORT = 8;

		
	//Drive Mode 
	public static final int ARCADE_DRIVE = 1;
	public static final int TANK_DRIVE = 0;
	public static int DRIVE_MODE = 1; // 1 is arcade drive, 0 is tank drive
	
	//PID variable
		public static boolean DRIVETRAIN_PID = true;

   // Variables used to tune velocity PID
   // TODO recheck this number to be sure it represents
   // 12 volt max power since we set the max power to 0.5 above.
   // For Kongo max speed is 6840 u/100ms
   // This is 68400 units per second
   // This means max speed 12v is at Velocity set in u/100ms = 6840
	public static final boolean TUNE_VEL_PID = false;
	// Make 3000 the max speed to give Batt and motor some overhead and safety
	public static final double TUNE_VEL_SPEED = -3000; // neg = forward
	
	
	
	// Debug variables
	public static final boolean MASTER_DEBUG = true;
	@SuppressWarnings("unused")
	public static final boolean DRIVETRAIN_DEBUG = true || MASTER_DEBUG;

// TODO There should be an explanation as to how  the drive train circumference is measured.
// This is most likely the circumference of the robot as it pivots on one wheel. This would
// mean, at least for Kongo, the wheel base diameter X pi This should also state units.
	public static final double DRIVETRAIN_CIRCUMFERENCE = .04785; 
	
//  Automode constraints
	public static final double TIME_STEP = 0.05; // sec
	//All in meters 
    public static final double WHEELBASE_WIDTH = 0.4191; // in meters
	public static final double MAX_VELOCITY = 6.0;// meters/sec
	public static final double MAX_ACCELERATION = 3.0; // meters/s/s:
	public static final double MAX_JERK = 18.0; // meters/s/s/s 

	public static final boolean LEFT_SIDE_INVERTED = true; 
	public static final boolean RIGHT_SIDE_INVERTED = false; 
	public static final boolean LEFT_SIDE_SENSOR_PHASE_REVERSED = true;
	public static final boolean RIGHT_SIDE_SENSOR_PHASE_REVERSED = false;
	
	public static final double RIGHT_JOYSTICK_DEAD_ZONE = 0.25;
	public static final double LEFT_JOYSTICK_DEAD_ZONE = 0.25;
	public static final double RIGHT_JOYSTICK_REDUCTION = 0.25;
	public static final double LEFT_JOYSTICK_REDUCTION = 0.25;

	
	// Settings for motor controllers
	public static final int DRIVETRAIN_PEAK_CURRENT = 40;
	
	// Drivetrain PID settings
	// TODO Good no load PID settings for now. Fine tune more as 
	// I get positional PID working. 
	// TODO We may want to make each side separate
	public static double DRIVETRAIN_KP = 0.3;
	public static double DRIVETRAIN_KI = 0.05;
	public static double DRIVETRAIN_KD = 0.03;
	public static double DRIVETRAIN_KF = 0.161; 
	// Integral Zone The motor control profile contains Integral Zone (I Zone), 
	// which (when nonzero), is the maximum error where Integral Accumulation will occur
	// during a closed-loop Mode. If the Closed-loop error is outside of the I Zone,
	// “I Accum” is automatically cleared. This can prevent total instability due to 
	// integral windup, particularly when tweaking gains.
	// The units are in the same units as the selected feedback device 
	//(Quadrature Encoder, Analog Potentiometer, Analog Encoder, and EncRise).
	public static int DRIVETRAIN_IZONE = 50;
	// All speeds will be set as a % of an absolute MAX_SPEED
	// in order to set both int fields and double fields we use the Double Class
	// rather than just a variable double. This way we can convert it as needed.
	// TODO verify units for all the below
	public static Double MAX_SPEED = 3000.0;

	// Set motion cruise velocity in RPM must be an int
	public static Double MOTION_CRUISE_VELOCITY = MAX_SPEED * 0.1;
	// In Arcade mode we may need to swap the direction of zRotation
	// if the robot spins opposite the joystick x direction
	public static final boolean INVERT_ZROTATION = true;

	// I use this to block user input whenever I triger a command from the 
	// smartdashboard
	public static boolean BLOCK_JOYSTICK_INPUT = false;
	
	// Used for MotionProfileOnboardRunner
		public static final double WHEEL_DIAMETER = 0.1524;// in meters 
		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
		public static final double METERS_PER_ROTATION = 1/WHEEL_CIRCUMFERENCE;
		// Encoders on Kongo are set for 2048 ppr. When we select quad 
		// encoder mode we get 2048*4 or 8192 units per revolution
		public static final int UNITS_PER_REVOLUTION = 8192;
		// This is measured using the web interface see sec 6.1 ot the Talon MP reference manual 
		public static final double MAX_UNITS_PER_100MS = 50490;
		//EP Some of these are not used right now. These are not limits.
		public static final double MAX_UNITS_PER_SECOND = MAX_UNITS_PER_100MS * 10;
		public static final double MAX_REVOLUTIONS_PER_SECOND = MAX_UNITS_PER_SECOND / UNITS_PER_REVOLUTION;
		public static final double MAX_METERS_PER_SECOND = WHEEL_CIRCUMFERENCE * MAX_REVOLUTIONS_PER_SECOND;
		
}

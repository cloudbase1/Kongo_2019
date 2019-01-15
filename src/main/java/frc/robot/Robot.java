/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* Kongo 2018 */

package frc.robot;

//import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.automodes.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.trajectories.TestTrajectory;
import frc.trajectories.Path;
import frc.robot.ADIS16448_IMU;

import frc.trajectories.MotionProfileRunner;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static final DriveTrainPID driveTrain = new DriveTrainPID();
	public static OI oi;
	//public static CameraServer cam;
	//public static final Encoder testEncoder = new Encoder(0, 1);
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	public static final ADIS16448_IMU gyro = new ADIS16448_IMU();
	public static final Path testTrajectory = new TestTrajectory();
	public static MotionProfileRunner motionProfileRunner = new MotionProfileRunner(Robot.driveTrain.left, Robot.driveTrain.right);


	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	@SuppressWarnings("static-access")
	@Override
	public void robotInit() {
		oi = new OI();
		//CameraServer.getInstance().startAutomaticCapture();
		//SmartDashboard.putData("Drive Straight one meter", new AutoDriveStraight());
		SmartDashboard.putData("Gyro TurnRight90 ", new TurnRight90());
		//SmartDashboard.putData("FirstMPAutoTest ", new FirstMPAutoTest());
		
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		driveTrain.stop();
		motionProfileRunner.reset();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		driveTrain.stop();
	

	}
		
	 public void robotPeriodic(){
		   // MPR control Needs to be called every cycle
		   motionProfileRunner.control();
    	if (RobotMap.DRIVETRAIN_DEBUG ){
		    SmartDashboard.putNumber("Heartbeat <3 ", Math.random());
	    	SmartDashboard.putNumber("DriveTrain/Left Speed", Robot.driveTrain.getLeftSpeed());
	    	SmartDashboard.putNumber("DriveTrain/Right Speed", Robot.driveTrain.getRightSpeed());
	    	SmartDashboard.putNumber("DriveTrain/Left Position", Robot.driveTrain.getLeftPos());
	    	SmartDashboard.putNumber("DriveTrain/Right Position", Robot.driveTrain.getRightPos());
	        SmartDashboard.putNumber("DriveTrain/Average Error", Robot.driveTrain.avgErr());
	    	SmartDashboard.putNumber("DriveTrain/Average Speed", Robot.driveTrain.avgSpeed());
	    	SmartDashboard.putNumber("Right Sensor Velocity", Robot.driveTrain.right.getSelectedSensorVelocity(0) * -1);
	    	SmartDashboard.putNumber("Left Sensor Velocity", Robot.driveTrain.left.getSelectedSensorVelocity(0) * -1);
	    	SmartDashboard.putString("DriveTrain Mode", Robot.driveTrain.getMode());
	    	
	    	// Gyro module smart dashboard display
	    	 SmartDashboard.putNumber("Gyro-X", gyro.getAngleX());
	    	 SmartDashboard.putNumber("Gyro-Y", gyro.getAngleY());
	    	 SmartDashboard.putNumber("Gyro-Z", gyro.getAngleZ());
	    	    
	    	 SmartDashboard.putNumber("Accel-X", gyro.getAccelX());
	    	 SmartDashboard.putNumber("Accel-Y", gyro.getAccelY());
	    	 SmartDashboard.putNumber("Accel-Z", gyro.getAccelZ());
	    	    
	    	 SmartDashboard.putNumber("Pitch", gyro.getPitch());
	    	 SmartDashboard.putNumber("Roll", gyro.getRoll());
	    	 SmartDashboard.putNumber("Yaw", gyro.getYaw());
	    	    
	    	 SmartDashboard.putNumber("Pressure: ", gyro.getBarometricPressure());
	    	 SmartDashboard.putNumber("Temperature: ", gyro.getTemperature()); 
     
   	}

    	
	    	// One of the features of commands is that it allows the program to be broken down into separate 
	    	// testable units. Each command can be run independently of any of the others. By writing commands 
	    	// to the SmartDashboard, they will appear on the screen as buttons that, when pressed, 
	    	// schedule the particular command. This allows any command to be tested individually by pressing 
	    	// the button and observing the operation.
	    	//SmartDashboard.putData("Drive Straight ", new AutoDriveStraight());
    	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		gyro.reset();
		driveTrain.resetEncoders();
		motionProfileRunner.control();
		//int auto = (int) SmartDashboard.getNumber("DB/Slider 0", 0);
		// TODO Hard code auto for now Maybe make this a field rather than a slider.
		int auto = 2;
		switch (auto){
		case 0:
    	    System.out.println("Robot: AutonomousCommand is null");
			autonomousCommand = null;
			break;
		case 1:
    	    System.out.println("Robot: Selecting AutoDriveStraight");
			autonomousCommand = new AutoDriveStraight();
			break;
		case 2:
    	    System.out.println("Robot: Selecting FirstMPAutoTest");
			autonomousCommand = new FirstMPAutoTest();
			break;
		case 3:
    	    //System.out.println("Robot: Selecting TurnGyro 90");
			//autonomousCommand = new TurnRight90();
			break;
		}
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
	       autonomousCommand.start();
        }

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	 Scheduler.getInstance().run();
 	 motionProfileRunner.control();
	}

	@SuppressWarnings("unused")
	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		if (RobotMap.DRIVE_MODE == 1) (new ArcadeDrive()).start();
		else (new TankDrive()).start();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}

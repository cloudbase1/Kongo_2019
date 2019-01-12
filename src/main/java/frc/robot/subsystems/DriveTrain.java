/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* DriveTrain for Kongo_Ed. This is the most basic DriveTrain we use. It  */
/* will have both arcade and tank drive. We start with arcade sine I have one */
/* joystick. Kongo also only has two drive wheels.                            */

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TankDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	private WPI_TalonSRX right;
	private WPI_TalonSRX left;
	
	private DifferentialDrive robotDrive; 
	/********************************************************************************/
	/* DriveTrain is a subsystem that connects the software to the motor ports */
	/* driveTrain is a non-PID subsystem and is simpler to use, but less accurate, */
	/* than the PID subsystem. Only one can be used at a time.                     */
	/********************************************************************************/
	public DriveTrain() {
		right = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_PORT);
		left = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_PORT);
		
		
		// Puts motors into RobotDrive class
		robotDrive = new DifferentialDrive(left, right);
		invertLeftSide(RobotMap.LEFT_SIDE_INVERTED);
		invertRightSide(RobotMap.RIGHT_SIDE_INVERTED);

	}
/********************************************************************************/
/* Both tankDrive and arcadeDrive are commands that take the joystick input */
/* and translates it to motor power commands to the motor controllers.          */
/********************************************************************************/
	public void tankDrive(double leftPower, double rightPower) {
		robotDrive.tankDrive(leftPower, rightPower);
		if (RobotMap.DRIVETRAIN_DEBUG) {
			System.out.println("[Subsystem] Driving in tank mode, left: " + leftPower + ", right: " + rightPower);
		}
	}
	
	public void arcadeDrive (double leftPower, double rightPower) {
		robotDrive.arcadeDrive(leftPower, rightPower);
		if (RobotMap.DRIVETRAIN_DEBUG) {
			System.out.println("[Subsystem] Driving in arcade mode, left: " + leftPower + ", right: " + rightPower);
		}
	}
	
	public void invertRightSide(boolean invert) {
		right.setInverted(invert);
	}
	
	public void invertLeftSide(boolean invert) {
		left.setInverted(invert);
	}

	/* The default command is needed to tell the subsystem what to do if there */
	/* is no input from the driver. We don't want random activity.                 */
	/* In this case the as a default command we set the drive mode.                */
	@SuppressWarnings("unused")
	public void initDefaultCommand() {
		if (RobotMap.DRIVE_MODE == 1){
			setDefaultCommand(new ArcadeDrive());
		}
		else{
			setDefaultCommand(new TankDrive());
		}
	}
	
	public void setLeftMotorPower(double power) {
		if (power > 1) {
			if (RobotMap.DRIVETRAIN_DEBUG) {
				System.out.println("[Subsystem] Power to left side of drivetrain was set too high: " + power + ", changing to full forward.");
			}
			power = 1;
		} else if (power < -1) {
			if (RobotMap.DRIVETRAIN_DEBUG) {
				System.out.println("[Subsystem] Power to left side of drivetrain was set too low: " + power + ", changing to full reverse.");
			}
			power = -1;
		}
			left.set(power * RobotMap.MAX_MOTOR_POWER);
	}
	
	public void setRightMotorPower(double power) {
		if (power > 1) {
			if (RobotMap.DRIVETRAIN_DEBUG) {
				System.out.println("[Subsystem] Power to right side of drivetrain was set too high: " + power + ", changing to full forward.");
			}
			power = 1;
		} else if (power < -1) {
			if (RobotMap.DRIVETRAIN_DEBUG) {
				System.out.println("[Subsystem] Power to right side of drivetrain was set too low: " + power + ", changing to full reverse.");
			}
			power = -1;
		}
	    right.set(power * RobotMap.MAX_MOTOR_POWER);
		
	}
	/* This method provides a way to stop the drive in an emergency. */
		public void stop() {
			left.set(ControlMode.PercentOutput, 0);
			right.set(ControlMode.PercentOutput, 0);
	}
}


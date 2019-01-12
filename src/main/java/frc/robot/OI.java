/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.commands.*;
//import frc.robot.triggers.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
/* For Kongo we assume where there is one joystick we are using all left */
/* joystick variables and right is ignored.                                    */
public class OI {
	private static Joystick leftStick = new Joystick (RobotMap.LEFT_JOYSTICK_PORT);
	private static Joystick rightStick = new Joystick (RobotMap.RIGHT_JOYSTICK_PORT);
	
	
	public OI () {

		
	}
	public double getRightStickY() {
		
		double rightStickY = rightStick.getY(); 
		
		if (Math.abs(rightStick.getMagnitude()) < RobotMap.RIGHT_JOYSTICK_DEAD_ZONE) {
			rightStickY = 0;
		}
		
		 return rightStickY;
		
	}
	
	public double getRightStickX() {
		
		double rightStickX = rightStick.getX(); 
		
		// Dead zone management
		if (Math.abs(rightStick.getMagnitude()) < RobotMap.RIGHT_JOYSTICK_DEAD_ZONE) {
			rightStickX = 0;
		}
		
		return rightStickX;
		
	}

	public double getLeftStickY() {
		
		double leftStickY = leftStick.getY(); 
		
		// Dead zone management
		if (Math.abs(leftStick.getMagnitude()) < RobotMap.LEFT_JOYSTICK_DEAD_ZONE) {
			leftStickY = 0;
		}
		
		return leftStickY;
		
	}
	
	public double getLeftStickX() {	
		double leftStickX = leftStick.getX(); 
		
		// Dead zone management
				if (Math.abs(leftStick.getMagnitude()) < RobotMap.LEFT_JOYSTICK_DEAD_ZONE) {
					leftStickX = 0;
				}		
				
		return leftStickX;
		
	}
	

	public boolean getRightButton(int button){
		return rightStick.getRawButton(button);
	}
	
	public double getDashboardDelaySlider() {
		return SmartDashboard.getNumber("DB/Slider 1", 0);
	}
    public double getDashboardSlider3() {
        return SmartDashboard.getNumber("DB/Slider 3", 0);
    }


/// TODO Is OI the proper place for these functions? Copies from 2018 OI
	/**
	 * Converts meters to encoder units.
	 * So the users enters meters and gets back the number of 
	 * native units to send to the talons.
	 * Uses {@value #WHEEL_DIAMETER}" for wheel diameter and {@value #UNITS_PER_REVOLUTION} for encoder units per revolution.
	 * WHEEL_DIAMETER is assumed to be in meeters
	 * @param meter
	 * @return encoder units
	 */
	public static double meter2Units(double meter) {
		meter /= RobotMap.WHEEL_CIRCUMFERENCE; // revolutions
		meter *= RobotMap.UNITS_PER_REVOLUTION; // Units
		return meter;
	}

	/**
	 * Converts encoder units into meters.
	 * The user enters the number of talon native units 
	 * and gets back the number of meters that represents.
	 * Uses {@value #WHEEL_DIAMETER}" for wheel diameter and {@value #UNITS_PER_REVOLUTION} for encoder units per revolution.
	 * @param meter
	 * @return encoder units
	 */
	public static double units2meters(double units) {
		units /= RobotMap.UNITS_PER_REVOLUTION; // revolutions
		units *= RobotMap.WHEEL_CIRCUMFERENCE;
		return units;
	}
	
	/**
	 * Converts meters per second to encoder units per 100 milliseconds.
	 * Uses {@value #WHEEL_DIAMETER}" for wheel diameter and {@value #UNITS_PER_REVOLUTION} for encoder units per revolution.
	 * @param mps  = meters per second
	 * @return encoder units per 100 milliseconds
	 */
	public static double mps2UnitsPerRev(double mps) {
		mps /= 10; // meters/100ms
		mps /= RobotMap.WHEEL_CIRCUMFERENCE; // revolutions/100ms
		mps *= RobotMap.UNITS_PER_REVOLUTION; // Units/100ms
		return mps;
	}
	

    
}

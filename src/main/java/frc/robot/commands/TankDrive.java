package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class TankDrive extends Command {
	/*  This command connects the joystick inputs pointed to by oi to the     */
	/* tank drive subsystem in the main robot class. This requires two joysticks.*/

	public TankDrive(){
		requires(Robot.driveTrain);
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}
	
	protected void execute(){
		Robot.driveTrain.tankDrive(Robot.oi.getLeftStickY(), Robot.oi.getRightStickY());
	}
	
	protected void stop(){
		Robot.driveTrain.tankDrive(0, 0);
	}

}

package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ArcadeDrive extends Command {
	/* This command connects the joystick inputs pointed to in the oi file     */
	/* with the drivetrain in the main robot class.   */ 
	public ArcadeDrive() {
		requires(Robot.driveTrain);
	}
	
	protected void execute() {
		Robot.driveTrain.arcadeDrive(Robot.oi.getLeftStickY() , Robot.oi.getLeftStickX()) ;
	}	  
	  
	protected void initialize() {}
	  
	protected void end() {}
	  
	protected void interrupted() {
		end();
	}
    
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}

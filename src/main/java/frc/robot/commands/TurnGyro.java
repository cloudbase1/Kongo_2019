package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnGyro extends Command {
	int angle;
	double error = 0;
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		System.out.println("TurnGyro: isFinished");
		if(Math.abs(this.error) < 25.0) {
			RobotMap.BLOCK_JOYSTICK_INPUT = false;
			return true;}
		else {
		return false;}
		
		
		
	}
	
	public TurnGyro(int angle){
		this.angle = angle;
	}
	
	protected void initialize(){
		//Robot.driveTrain.setSensorPhase(RobotMap.LEFT_SIDE_SENSOR_PHASE_REVERSED, RobotMap.RIGHT_SIDE_SENSOR_PHASE_REVERSED);
		System.out.println("TurnGyro: Initalize Resetting encoders and gyro");
		Robot.gyro.reset();
		Robot.driveTrain.resetEncoders();

	}
	protected void execute(){
		
		//RobotMap.BLOCK_JOYSTICK_INPUT = true;
		System.out.println("TurnGyro: Execute");
		this.error = Robot.driveTrain.goToAngle(angle);
	}
	   // Called once after isFinished returns true
    protected void end() {
    	System.out.println("TrunGyro: Ending");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("TrunGyro: Interrupted");
    }
}

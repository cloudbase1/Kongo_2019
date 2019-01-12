package frc.robot.commands;

import java.io.BufferedWriter;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMagic extends Command {

	int left;
	int right;
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		if(Robot.driveTrain.nearGoal()) {
			RobotMap.BLOCK_JOYSTICK_INPUT = false;}
		return Robot.driveTrain.nearGoal();
		
		
	}
	
	public DriveMagic(int left, int right){
		this.left = left;
		this.right = right;
	}
	
	protected void initialize(){
		Robot.driveTrain.setSensorPhase(RobotMap.LEFT_SIDE_SENSOR_PHASE_REVERSED, RobotMap.RIGHT_SIDE_SENSOR_PHASE_REVERSED);
		Robot.driveTrain.resetEncoders();
		

	}
	protected void execute(){
		RobotMap.BLOCK_JOYSTICK_INPUT = true;
		Robot.driveTrain.magicDrive(left, right);
		SmartDashboard.putNumber("Left Speed", Robot.driveTrain.getLeftSpeed());
		SmartDashboard.putNumber("Right Speed", Robot.driveTrain.getRightSpeed());
		SmartDashboard.putNumber("Left Pos", Robot.driveTrain.getLeftPos());
		SmartDashboard.putNumber("Right Pos", Robot.driveTrain.getRightPos());
		SmartDashboard.putNumber("Left Error", Robot.driveTrain.getLeftError());
		SmartDashboard.putNumber("Right Error", Robot.driveTrain.getRightError());
		//SmartDashboard.putNumber("Gyro Angle", Robot.driveTrain.getGyroAngle());
		//SmartDashboard.putNumber("Gyro Rate", Robot.driveTrain.getGyroRate());
		SmartDashboard.putNumber("Correction Value", 0);
		SmartDashboard.putNumber("Avg Speed", Robot.driveTrain.avgSpeed());
	}
	   // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

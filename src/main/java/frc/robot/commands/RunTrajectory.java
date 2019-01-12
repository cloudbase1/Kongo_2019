package frc.robot.commands;

import frc.trajectories.MotionProfileRunner;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunTrajectory extends Command {

	MotionProfileRunner.TrajectorySelect trajectory;
	
    public RunTrajectory(MotionProfileRunner.TrajectorySelect trajectory) {
    	this.trajectory = trajectory;
    	requires(Robot.driveTrain);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.setSensorPhase(RobotMap.LEFT_SIDE_SENSOR_PHASE_REVERSED, RobotMap.RIGHT_SIDE_SENSOR_PHASE_REVERSED);
    	Robot.motionProfileRunner.setCurrentTrajectory(trajectory);
		SetValueMotionProfile setOutput = Robot.motionProfileRunner.getSetValue();
    	System.out.println("RunTrajectory: Initialize() trajectory: " + trajectory.toString());
    	System.out.println("RunTrajectory: Initialize() SetOutput: " + setOutput.toString());
		Robot.driveTrain.left.set(ControlMode.MotionProfile, setOutput.value);
		Robot.driveTrain.right.set(ControlMode.MotionProfile, setOutput.value);
		Robot.motionProfileRunner.startMotionProfile();
		
    	setTimeout(0.25);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		SetValueMotionProfile setOutput = Robot.motionProfileRunner.getSetValue();
    	System.out.println("RunTrajectory: Execute() SetOutput: " + setOutput.toString());
		Robot.driveTrain.left.set(ControlMode.MotionProfile, setOutput.value);
		Robot.driveTrain.right.set(ControlMode.MotionProfile, setOutput.value);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("RunTrajectory: Checking isFinished for trajectory: " + trajectory.toString());
        return Robot.motionProfileRunner.getSetValue() == SetValueMotionProfile.Disable && isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("RunTrajectory: Ending trajectory: " + trajectory.toString());
    	Robot.motionProfileRunner.reset();
    	Robot.driveTrain.resetEncoders();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}

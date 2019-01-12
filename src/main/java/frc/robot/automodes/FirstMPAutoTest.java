package frc.robot.automodes;


import frc.robot.commands.RunTrajectory;
import frc.trajectories.MotionProfileRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class FirstMPAutoTest extends CommandGroup {
	public FirstMPAutoTest() {
		double delay = 1;// TODO Hard code for now Robot.oi.getDashboardDelaySlider();
        addSequential(new WaitCommand(delay));
        // RunTrajectory sinks the MPR and talon states
		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.TestTrajectory
				));
	}
}


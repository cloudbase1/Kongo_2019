package frc.robot.automodes;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

import frc.robot.commands.*;
//import frc.robot.*;

/**
 *
 */
public class TurnRight90 extends CommandGroup {

    public TurnRight90() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	System.out.println("TurnRight90: Posting TurnGyro 90");
    	addSequential(new TurnGyro(-90));
    	// wait preassigned time
    	System.out.println("TurnRight90: Waiting one second");
    	addSequential(new WaitCommand(1));
    }
}

// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2538.RecycleRush2538.commands;
import org.usfirst.frc2538.RecycleRush2538.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SmartStack extends CommandGroup {
    
    public  SmartStack() {
    	
    	// doesn't work because constructors only run once
    	/*
    	if (Robot.intake.isFirstThing) {
			new GetThing();
		}
    	else{
    		new StackThing();
    	}
    	*/
    	addSequential(new ToteConfigCommand());
    	addSequential(new intakeCommand());
    	addSequential(new ContainerConfigCommand());
    	addSequential(new ToStackOrNotStackCommand());
    	addSequential(new RaiseLift());
    	addSequential(new Wait(), .25);
    	addSequential(new RetractTridentCommand());
    	
    	
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
    }
}

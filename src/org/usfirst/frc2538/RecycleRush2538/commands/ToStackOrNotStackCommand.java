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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc2538.RecycleRush2538.Robot;
import org.usfirst.frc2538.RecycleRush2538.subsystems.OurTimer;

/**
 *
 */
public class  ToStackOrNotStackCommand extends Command {

    public ToStackOrNotStackCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.lift);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    private boolean isDone = false;
    private boolean loweredLift = false;
    private boolean extendedTrident = false;
    private OurTimer timer = new OurTimer();
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	isDone = false;
    	loweredLift = false;
    	extendedTrident = false;
    	
    	if (Robot.intake.isFirstThing) {
    		Robot.intake.isFirstThing = false;
			isDone = true;
		}
    	else {
    		timer.restart();
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//SmartDashboard.putString("2Stack", "executing");
    	SmartDashboard.putBoolean("loweredLift", loweredLift);
    	long time = timer.getElapsedTime();
    	if (!isDone) {
    		//SmartDashboard.putString("2Stack", "stack");
    		//SmartDashboard.putBoolean("loweredLift", loweredLift);
    		if (!loweredLift) {
				Robot.lift.lowerLift();
    			Robot.lift.lifting.set(DoubleSolenoid.Value.kForward);				
			} 
    		else if (time > 100) {
    			SmartDashboard.putString("2Stack", "done lowering lift");
    			loweredLift = true;
    		}
    		else if (loweredLift && !extendedTrident) {
    			SmartDashboard.putString("2Stack", "extend trident");
    			Robot.lift.extendTrident();
    		}
    		else if(time > 200) {
    			SmartDashboard.putString("2Stack", "done extending");
    			extendedTrident = true;
    		}
    		else if (Robot.intake.okay()) {
    			SmartDashboard.putString("2Stack", "intake");
    			Robot.intake.limitSwitchIntake();
    		}
    		else {
    			SmartDashboard.putString("2Stack", "done");
    			isDone = true;
    		}
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isDone;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
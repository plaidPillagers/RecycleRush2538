// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2538.RecycleRush2538.subsystems;

import org.usfirst.frc2538.RecycleRush2538.RobotMap;
import org.usfirst.frc2538.RecycleRush2538.commands.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class Test extends Subsystem {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SpeedController test = RobotMap.testtest;
    Encoder testEncoder = RobotMap.testtestEncoder;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new testTalon());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    

	public void setTestController() {
		Preferences.getInstance().remove("test talon");
    	SmartDashboard.putNumber("test talon", 0.0);
    	Preferences.getInstance().getDouble("test talon", 0.0);
    	Preferences.getInstance().save();
    	test.set(Preferences.getInstance().getDouble("test talon", 0.0));
    	SmartDashboard.putNumber("test talon output", test.get());
 
	}
	
	public boolean testDistance() {
		SmartDashboard.putNumber("encoderDistance", testEncoder.getDistance());
		if (testEncoder.getDistance() > 184 && testEncoder.getDistance() < 192) {
			test.set(0);
			return true;
		}
		else {
			test.set(1);
			return false;
		}
	}
	
	public void testRate() {
		test.set(1);
		SmartDashboard.putNumber("encoderRate", testEncoder.getRate());
	}
}


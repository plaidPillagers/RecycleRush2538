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


import org.usfirst.frc2538.RecycleRush2538.Robot;
import org.usfirst.frc2538.RecycleRush2538.RobotMap;
import org.usfirst.frc2538.RecycleRush2538.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




/**
 *
 */
public class DriveSystem extends Subsystem {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SpeedController leftFront = RobotMap.driveSystemleftFront;
    SpeedController rightFront = RobotMap.driveSystemrightFront;
    SpeedController leftRear = RobotMap.driveSystemleftRear;
    SpeedController rightRear = RobotMap.driveSystemrightRear;
    RobotDrive robotDrive41 = RobotMap.driveSystemRobotDrive41;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private double JOYSTICK_TOLERANCE = .05;
    public boolean isInverted = false;
    public boolean isTwisted = false;
    
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new DriveMecnum());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void driveMecnum() {
    	Joystick driveJoystick = Robot.oi.driveStick;
    	double magnitude = driveJoystick.getMagnitude();
    	double direction = driveJoystick.getDirectionDegrees();
    	double rotation = driveJoystick.getZ();
    	double throttle = driveJoystick.getThrottle() * (-.25) + .75;
    	robotDrive41.mecanumDrive_Polar(minimumTolerance(magnitude) * throttle, reverseDirection(direction), checkTwist(reverseRotation(rotation)));
    	//stop multiplying by zero on rotation
    	displayDriveInfo(driveJoystick);
    	SmartDashboard.putNumber("Real magnitude: ", minimumTolerance(magnitude) * throttle);
    }
    
    private double minimumTolerance(double magnitude) {
    	if (magnitude < JOYSTICK_TOLERANCE) {
			return 0;
		}
    	return magnitude;
    }
    
    private double checkTwist(double rotation) {
    	if (isTwisted) {
    		return rotation;
    	}
    	return 0;
    }
    
    private double reverseDirection(double direction) {
    	if (isInverted) {
    		if (direction < 180) {
				return direction + 180;
			}
    		return direction - 180;
		}
    	return direction;
    }
    
    private double reverseRotation(double rotation) {
    	if (isInverted) {
			return -rotation;
		}
    	return rotation;
    }
    
    private void displayDriveInfo(Joystick driveStick) {
    	double joyStickX = driveStick.getX();
    	double joyStickY = driveStick.getY();
    	double joyStickZ = driveStick.getZ();
    	
    	double leftFrontVal = leftFront.get();
    	double leftRearVal = leftRear.get();
    	double rightFrontVal = rightFront.get();
    	double rightRearVal = rightRear.get();
    	
    	SmartDashboard.putNumber("Joystick X: ", joyStickX);
    	SmartDashboard.putNumber("Joystick Y: ", joyStickY);
    	SmartDashboard.putNumber("Joystick Z: ", joyStickZ);
    	
    	SmartDashboard.putNumber("Left Front Motor: ", leftFrontVal);
    	SmartDashboard.putNumber("Left Rear Motor: ", leftRearVal);
    	SmartDashboard.putNumber("Right Front Motor: ", rightFrontVal);
    	SmartDashboard.putNumber("Right Rear Motor: ", rightRearVal);
    	
    }
    
}


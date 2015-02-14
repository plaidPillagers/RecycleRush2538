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

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intake extends Subsystem {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SpeedController leftWheel = RobotMap.intakeleftWheel;
    SpeedController rightWheel = RobotMap.intakerightWheel;
    DigitalInput containerSwitch = RobotMap.intakecontainerSwitch;
    DigitalInput toteSwitch = RobotMap.intaketoteSwitch;
    DoubleSolenoid configActuator = RobotMap.intakeconfigActuator;
    Encoder leftIntakeEncoder = RobotMap.intakeleftIntakeEncoder;
    Encoder rightIntakeEncoder = RobotMap.intakerightIntakeEncoder;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	private final double INTAKE_SPEED = 1;
	private final double EJECT_SPEED = -1;
	private final double LEFT_SPIN = .7;
	private final double RIGHT_SPIN = -.7;
	private boolean isToteConfig = true;
	public boolean isFirstThing = true;
	

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new DisplayIntakeInfoCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void intake() {
		leftWheel.set(INTAKE_SPEED);
		rightWheel.set(-INTAKE_SPEED);
	}

	public void eject() {
		SmartDashboard.putString("eject method", "eject called");
		leftWheel.set(EJECT_SPEED);
		rightWheel.set(-EJECT_SPEED);
		displayEncoders();
	}

	public void stop() {
		leftWheel.set(0);
		rightWheel.set(0);
	}

	public void spin() {
		// spinning clockwise
		leftWheel.set(LEFT_SPIN);
		rightWheel.set(-RIGHT_SPIN);
		displayEncoders();
	}
	
	public void setConfig() {
		if (isToteConfig) {
			configActuator.set(DoubleSolenoid.Value.kForward);
			isToteConfig = false;
		}
		else {
			configActuator.set(DoubleSolenoid.Value.kReverse);
			isToteConfig = true;
		}
	}

	// these methods insure correct booleans depending on how the switch is set
	// this method returns true if its okay to move and false otherwise
	public boolean okay() {
		if (isToteConfig) {
			if (toteSwitch.get()) {
				return true;
			}
			return false;
		} else if (toteSwitch.get()) {
			return true;
		}
		return false;
	}

	public void limitSwitchIntake() {
		// SmartDashboard.putBoolean("containerClear: ", okayContainer());
		// SmartDashboard.putBoolean("toteClear: ", okayTote());
		if (okay()) {
			intake();
		} else {
			stop();
		}
		displayEncoders();
	}
	
	public void displayEncoders() {
		double leftIntakeSpeed = Robot.intake.leftIntakeEncoder.getRate();
		double rightIntakeSpeed = Robot.intake.rightIntakeEncoder.getRate();
		
		SmartDashboard.putNumber("Left intake encoder", leftIntakeSpeed);
		SmartDashboard.putNumber("Right intake encoder", rightIntakeSpeed);
	}
	
	public void conatinerConfigure() {
		configActuator.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void toteConfigure() {
		configActuator.set(DoubleSolenoid.Value.kForward);
	}
	
	public void runIntakeWheels() {
		leftWheel.set(INTAKE_SPEED);
		rightWheel.set(-INTAKE_SPEED);
	}
}

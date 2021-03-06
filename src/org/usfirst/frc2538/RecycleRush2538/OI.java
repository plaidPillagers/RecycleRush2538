// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2538.RecycleRush2538;

import org.usfirst.frc2538.RecycleRush2538.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);
    
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.
    
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());

    
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public JoystickButton invertDriveButton;
    public JoystickButton getThingDriverButton;
    public JoystickButton forwardHeavyButton;
    public JoystickButton ejectButton;
    public JoystickButton aftHeavyButton;
    public JoystickButton stackThingDriverButton;
    public JoystickButton twistButton;
    public JoystickButton intakeButton;
    public JoystickButton balancedButton;
    public Joystick driveStick;
    public JoystickButton configButton;
    public JoystickButton tridentActuatorButton;
    public JoystickButton manualStackThingButton;
    public JoystickButton liftAndRetractButton;
    public JoystickButton intakeToggleButton;
    public JoystickButton liftActuatorButton;
    public JoystickButton ejectOnly;
    public JoystickButton ejectGroupButton;
    public JoystickButton stopButton;
    public JoystickButton containerLiftButton;
    public JoystickButton spinCounterClockwiseButton;
    public JoystickButton spinClockwiseButton;
    public JoystickButton intakeWheelsOnly;
    public JoystickButton stackThingButton;
    public JoystickButton getThingButton;
    public JoystickButton diagnostics;
    public JoystickButton testEncoderRateButton;
    public JoystickButton testResetButton;
    public Joystick secondaryStick;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        secondaryStick = new Joystick(1);
        
        testResetButton = new JoystickButton(secondaryStick, 16);
        testResetButton.whenPressed(new TestReset());
        testEncoderRateButton = new JoystickButton(secondaryStick, 14);
        testEncoderRateButton.whenPressed(new TestEncoderDistance());
        diagnostics = new JoystickButton(secondaryStick, 16);
        diagnostics.whenPressed(new DisplayRangeFinder());
        getThingButton = new JoystickButton(secondaryStick, 15);
        getThingButton.whenPressed(new GetThing());
        stackThingButton = new JoystickButton(secondaryStick, 16);
        stackThingButton.whenPressed(new StackThing());
        intakeWheelsOnly = new JoystickButton(secondaryStick, 14);
        intakeWheelsOnly.whileHeld(new RunIntake());
        spinClockwiseButton = new JoystickButton(secondaryStick, 15);
        spinClockwiseButton.whileHeld(new SpinClockwiseCommand());
        spinCounterClockwiseButton = new JoystickButton(secondaryStick, 15);
        spinCounterClockwiseButton.whileHeld(new SpinCounterClockwiseCommand());
        containerLiftButton = new JoystickButton(secondaryStick, 15);
        containerLiftButton.whenPressed(new ContainerLift());
        stopButton = new JoystickButton(secondaryStick, 9);
        stopButton.whenPressed(new stop());
        ejectGroupButton = new JoystickButton(secondaryStick, 10);
        ejectGroupButton.whenPressed(new EjectGroup());
        ejectOnly = new JoystickButton(secondaryStick, 7);
        ejectOnly.whileHeld(new eject());
        liftActuatorButton = new JoystickButton(secondaryStick, 3);
        liftActuatorButton.whenPressed(new liftActuator());
        intakeToggleButton = new JoystickButton(secondaryStick, 8);
        intakeToggleButton.whenPressed(new IntakeWheelsToggle());
        liftAndRetractButton = new JoystickButton(secondaryStick, 6);
        liftAndRetractButton.whenPressed(new LiftAndRetract());
        manualStackThingButton = new JoystickButton(secondaryStick, 4);
        manualStackThingButton.whenPressed(new ManualStackThingGroup());
        tridentActuatorButton = new JoystickButton(secondaryStick, 5);
        tridentActuatorButton.whenPressed(new tridentActuator());
        configButton = new JoystickButton(secondaryStick, 1);
        configButton.whenPressed(new ConfigCommand());
        driveStick = new Joystick(0);
        
        balancedButton = new JoystickButton(driveStick, 12);
        balancedButton.whenPressed(new SetBalanced());
        intakeButton = new JoystickButton(driveStick, 13);
        intakeButton.whenPressed(new SmartStack());
        twistButton = new JoystickButton(driveStick, 7);
        twistButton.whileHeld(new twist());
        stackThingDriverButton = new JoystickButton(driveStick, 6);
        stackThingDriverButton.whenPressed(new StackThing());
        aftHeavyButton = new JoystickButton(driveStick, 5);
        aftHeavyButton.whenPressed(new SetAft());
        ejectButton = new JoystickButton(driveStick, 4);
        ejectButton.whenPressed(new EjectGroup());
        forwardHeavyButton = new JoystickButton(driveStick, 3);
        forwardHeavyButton.whenPressed(new SetForward());
        getThingDriverButton = new JoystickButton(driveStick, 2);
        getThingDriverButton.whenPressed(new GetThing());
        invertDriveButton = new JoystickButton(driveStick, 1);
        invertDriveButton.whileHeld(new invertDrive());

	    
        // SmartDashboard Buttons
        SmartDashboard.putData("DisplayRangeFinder", new DisplayRangeFinder());


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
    
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getdriveStick() {
        return driveStick;
    }

    public Joystick getsecondaryStick() {
        return secondaryStick;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}


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
    public JoystickButton twistButton;
    public JoystickButton intakeButton;
    public JoystickButton balancedButton;
    public JoystickButton forwardHeavyButton;
    public JoystickButton aftHeavyButton;
    public JoystickButton ejectButton;
    public Joystick driveStick;
    public JoystickButton liftActuatorButton;
    public JoystickButton configButton;
    public JoystickButton tridentActuatorButton;
    public JoystickButton stopButton;
    public JoystickButton spinButton;
    public JoystickButton getThingButton;
    public JoystickButton stackThingButton;
    public JoystickButton intakeWheelsOnly;
    public Joystick secondaryStick;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        secondaryStick = new Joystick(1);
        
        intakeWheelsOnly = new JoystickButton(secondaryStick, 9);
        intakeWheelsOnly.whileHeld(new RunIntake());
        stackThingButton = new JoystickButton(secondaryStick, 9);
        stackThingButton.whenPressed(new StackThing());
        getThingButton = new JoystickButton(secondaryStick, 10);
        getThingButton.whenPressed(new GetThing());
        spinButton = new JoystickButton(secondaryStick, 2);
        spinButton.whileHeld(new spin());
        stopButton = new JoystickButton(secondaryStick, 12);
        stopButton.whenPressed(new stop());
        tridentActuatorButton = new JoystickButton(secondaryStick, 5);
        tridentActuatorButton.whenPressed(new tridentActuator());
        configButton = new JoystickButton(secondaryStick, 3);
        configButton.whenPressed(new ConfigCommand());
        liftActuatorButton = new JoystickButton(secondaryStick, 6);
        liftActuatorButton.whenPressed(new liftActuator());
        driveStick = new Joystick(0);
        
        ejectButton = new JoystickButton(driveStick, 4);
        ejectButton.whenPressed(new EjectGroup());
        aftHeavyButton = new JoystickButton(driveStick, 2);
        aftHeavyButton.whenPressed(new SetAft());
        forwardHeavyButton = new JoystickButton(driveStick, 3);
        forwardHeavyButton.whenPressed(new SetForward());
        balancedButton = new JoystickButton(driveStick, 5);
        balancedButton.whenPressed(new SetBalanced());
        intakeButton = new JoystickButton(driveStick, 6);
        intakeButton.whenPressed(new SmartStack());
        twistButton = new JoystickButton(driveStick, 7);
        twistButton.whileHeld(new twist());
        invertDriveButton = new JoystickButton(driveStick, 1);
        invertDriveButton.whileHeld(new invertDrive());

	    
        // SmartDashboard Buttons

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


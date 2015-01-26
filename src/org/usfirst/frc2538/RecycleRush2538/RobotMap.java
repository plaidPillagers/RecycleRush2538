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
    

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.*;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveSystemleftFront;
    public static SpeedController driveSystemrightFront;
    public static SpeedController driveSystemleftRear;
    public static SpeedController driveSystemrightRear;
    public static RobotDrive driveSystemRobotDrive41;
    public static Ultrasonic driveSystemport;
    public static Ultrasonic driveSystemstarboard;
    public static SpeedController intakeleftWheel;
    public static SpeedController intakerightWheel;
    public static DigitalInput intakecontainerSwitch;
    public static DigitalInput intaketoteSwitch;
    public static DoubleSolenoid lifttrident;
    public static DoubleSolenoid liftlifting;
    public static SpeedController testtest;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveSystemleftFront = new Talon(2);
        LiveWindow.addActuator("DriveSystem", "leftFront", (Talon) driveSystemleftFront);
        
        driveSystemrightFront = new Talon(3);
        LiveWindow.addActuator("DriveSystem", "rightFront", (Talon) driveSystemrightFront);
        
        driveSystemleftRear = new Talon(1);
        LiveWindow.addActuator("DriveSystem", "leftRear", (Talon) driveSystemleftRear);
        
        driveSystemrightRear = new Talon(4);
        LiveWindow.addActuator("DriveSystem", "rightRear", (Talon) driveSystemrightRear);
        
        driveSystemRobotDrive41 = new RobotDrive(driveSystemleftFront, driveSystemleftRear,
              driveSystemrightFront, driveSystemrightRear);
        
        driveSystemRobotDrive41.setSafetyEnabled(true);
        driveSystemRobotDrive41.setExpiration(0.1);
        driveSystemRobotDrive41.setSensitivity(0.5);
        driveSystemRobotDrive41.setMaxOutput(1.0);

        driveSystemRobotDrive41.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        driveSystemRobotDrive41.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        driveSystemport = new Ultrasonic(2, 3);
        LiveWindow.addSensor("DriveSystem", "port", driveSystemport);
        
        driveSystemstarboard = new Ultrasonic(4, 5);
        LiveWindow.addSensor("DriveSystem", "starboard", driveSystemstarboard);
        
        intakeleftWheel = new Talon(5);
        LiveWindow.addActuator("Intake", "leftWheel", (Talon) intakeleftWheel);
        
        intakerightWheel = new Talon(6);
        LiveWindow.addActuator("Intake", "rightWheel", (Talon) intakerightWheel);
        
        intakecontainerSwitch = new DigitalInput(0);
        LiveWindow.addSensor("Intake", "containerSwitch", intakecontainerSwitch);
        
        intaketoteSwitch = new DigitalInput(1);
        LiveWindow.addSensor("Intake", "toteSwitch", intaketoteSwitch);
        
        lifttrident = new DoubleSolenoid(0, 0, 1);      
        LiveWindow.addActuator("Lift", "trident", lifttrident);
        
        liftlifting = new DoubleSolenoid(0, 2, 3);      
        LiveWindow.addActuator("Lift", "lifting", liftlifting);
        
        testtest = new Talon(8);
        LiveWindow.addActuator("Test", "test", (Talon) testtest);
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}

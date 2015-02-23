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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;


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
    Encoder leftFrontEncoder = RobotMap.driveSystemleftFrontEncoder;
    Encoder rightFrontEncoder = RobotMap.driveSystemrightFrontEncoder;
    Encoder leftRearEncoder = RobotMap.driveSystemleftRearEncoder;
    Encoder rightRearEncoder = RobotMap.driveSystemrightRearEncoder;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private MecanumSpeedConrtoller leftFrontMec;
    private MecanumSpeedConrtoller rightFrontMec;
    private MecanumSpeedConrtoller leftRearMec;
    private MecanumSpeedConrtoller rightRearMec;
    
    private byte[] buffer; //I don't know why need this but the read() method used by I2C really wants it
    private I2C forwardRangeFinder = new I2C(I2C.Port.kOnboard, 224);
    
    private double JOYSTICK_TOLERANCE = .05;
    public boolean isInverted = false;
    public boolean isTwisted = false;
    public Accelerometer accelerometer = new BuiltInAccelerometer();
    public double frontCompensation = 1.0;
    public double aftCompensation = 1.0;
    public double forwardDriveProportion = Robot.frontHeavy;
    public double balancedDriveProportion = Robot.balanced;
    public double aftDriveProportion = Robot.aftHeavy;
    public final int FORWARDDRIVE = 1;
    public final int BALANCEDDRIVE = 2;
    public final int AFTDRIVE = 3;
    public int driveMode  = BALANCEDDRIVE;
    MecanumSpeedConrtoller[] sketchySpeedConrtollers = new MecanumSpeedConrtoller[4];
    
    int WHEEL_RADIUS = 3;
    double pulsePerRotaion = 360;
    double driveEncoderDistancePerTick = (Math.PI * 2 * WHEEL_RADIUS) / pulsePerRotaion;
    long startTime = 0;
    
    double circularXVal = 0.0;
    double circularYVal = 0.0;
   
    /***
     * AUTO CONSTANTS
     */
    private final double DISTANCEFROMWALL = 56;
    private final double DISTANCETOTHING = 19;
    private final double ANGLEMARGIN = 10;
    
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new driveHomemadeMecanum());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void driveMecnum() {
    	Joystick driveJoystick = Robot.oi.driveStick;
    	double magnitude = driveJoystick.getMagnitude();
    	double direction = driveJoystick.getDirectionDegrees();
    	double rotation = driveJoystick.getThrottle();
    	double throttle = driveJoystick.getZ() * (-.25) + .75;
    	//madcatz joystick switches throttle and z
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
    	double joyStickZ = driveStick.getThrottle();
    	double throttle = driveStick.getZ() * (-.25) + .75;
    	double magnitude = driveStick.getMagnitude();
    	double angle = driveStick.getDirectionRadians();
    	
    	double leftFrontVal = leftFront.get();
    	double leftRearVal = leftRear.get();
    	double rightFrontVal = rightFront.get();
    	double rightRearVal = rightRear.get();
    	
    	double accelX = accelerometer.getX();
    	double accelY = accelerometer.getY();
    	double accelZ = accelerometer.getZ();
    	
    	double leftFrontEncoderSpeed = getEncoderSpeed(leftFrontEncoder);
    	double rightFrontEncoderSpeed = getEncoderSpeed(rightFrontEncoder);
    	double leftRearEncoderSpeed = getEncoderSpeed(leftRearEncoder);
    	double rightRearEncoderSpeed = getEncoderSpeed(rightRearEncoder);
    	
    	double frontLeftSpeed = leftFrontEncoder.getRate();
    	double frontRightSpeed = rightFrontEncoder.getRate();
    	double rearLeftSpeed = leftRearEncoder.getRate();
    	double rearRightSpeed = rightRearEncoder.getRate();
    	
    	// The built in I2C method returns true if not found and false if found (Don't ask why)
    	/*
    	try {
    		boolean forwardRangeFinderFound = !forwardRangeFinder.addressOnly();
		} catch (new NullPointerException()) {
			// TODO: handle exception
		}
		*/
    	
    	SmartDashboard.putNumber("Joystick X: ", joyStickX);
    	SmartDashboard.putNumber("Joystick Y: ", joyStickY);
    	SmartDashboard.putNumber("Joystick Z: ", joyStickZ);
    	SmartDashboard.putNumber("throttle", throttle);
    	
    	SmartDashboard.putNumber("Left Front Motor: ", leftFrontVal);
    	SmartDashboard.putNumber("Left Rear Motor: ", leftRearVal);
    	SmartDashboard.putNumber("Right Front Motor: ", rightFrontVal);
    	SmartDashboard.putNumber("Right Rear Motor: ", rightRearVal);
    	
    	SmartDashboard.putDouble("Accelerometer X", accelX);
    	SmartDashboard.putDouble("Accelerometer Y", accelY);
    	SmartDashboard.putDouble("Accelerometer Z", accelZ);
    	
    	SmartDashboard.putDouble("Front Left Speed", frontLeftSpeed);
    	SmartDashboard.putDouble("Front Right Speed", frontRightSpeed);
    	SmartDashboard.putDouble("Rear Left Speed", rearLeftSpeed);
    	SmartDashboard.putDouble("Rear Right Speed", rearRightSpeed);
    	
    	SmartDashboard.putNumber("circular x", circularXVal);
    	SmartDashboard.putNumber("circular y", circularYVal);
    	SmartDashboard.putNumber("joystick magnitude", magnitude);
    	SmartDashboard.putNumber("joystick angle", angle);
    	
    	SmartDashboard.putNumber("left front encoder speed: ", leftFrontEncoderSpeed);
    	SmartDashboard.putNumber("right front encoder speed: ", rightFrontEncoderSpeed);
    	SmartDashboard.putNumber("left rear encoder speed: ", leftRearEncoderSpeed);
    	SmartDashboard.putNumber("right rear encoder speed: ", rightRearEncoderSpeed);
    	
    	//SmartDashboard.putBoolean("ForwardRangeFinderFound", forwardRangeFinderFound);
    	
    }
    
    public void homemadeMecanum() {
    	
    	Joystick driveJoystick = Robot.oi.driveStick;
    	//circularJoystick(driveJoystick);
    	double joystickX = driveJoystick.getX();
    	double joystickY = - driveJoystick.getY();
    	double joystickZ = driveJoystick.getThrottle();
    	double throttle = driveJoystick.getZ() * (-.4) + .6;//throttles between .2 and 1
    	//madcatz joystick switches throttle and z
    	if (isTwisted) {
    		homemadeZ(joystickZ);
		}
    	else{
    		if (isInverted) {
    			setAll( -joystickX, -joystickY, throttle);
			}
    		else {
    			setAll(joystickX, joystickY, throttle);
			}
    	}
    	displayDriveInfo(driveJoystick);
    }

	public void makeWheels() {
		leftFrontMec = new MecanumSpeedConrtoller(leftFront, 1.0, 1.0, frontCompensation, false);
    	rightFrontMec = new MecanumSpeedConrtoller(rightFront, -1.0, 1.0, frontCompensation, false);
    	leftRearMec = new MecanumSpeedConrtoller(leftRear, -1.0, 1.0, aftCompensation, false);
    	rightRearMec = new MecanumSpeedConrtoller(rightRear, 1.0, 1.0, aftCompensation, false);
    	
    	sketchySpeedConrtollers[0] = leftFrontMec;
    	sketchySpeedConrtollers[1] = rightFrontMec;
    	sketchySpeedConrtollers[2] = leftRearMec;
    	sketchySpeedConrtollers[3] = rightRearMec;
    	
    	SmartDashboard.putString("make wheels", "making wheels");
	}
    
    public void setAll(double xVal, double yVal, double throttle) {
    	setDriveMode();
    	leftFrontMec.set(xVal, yVal, throttle);
    	rightFrontMec.set(xVal,yVal, -throttle);
    	leftRearMec.set(xVal, yVal, throttle);
    	rightRearMec.set(xVal, yVal, -throttle);
		
    }
    
    private void homemadeZ(double joystickZ) {
    	leftFrontMec.spinSet(joystickZ);
    	rightFrontMec.spinSet(joystickZ);
    	leftRearMec.spinSet(joystickZ);
    	rightRearMec.spinSet(joystickZ);
    }
    
    /**
     * 1 = forward drive mode
     * 2 = balanced drive mode
     * 3 = aft drive mode
     */
    private void setDriveMode() {
    	forwardDriveProportion = Robot.frontHeavy;
    	balancedDriveProportion = Robot.balanced;
    	aftDriveProportion = Robot.aftHeavy;
    	SmartDashboard.putNumber("front drive proportion", forwardDriveProportion);
    	
    	if (driveMode == FORWARDDRIVE) {
			frontCompensation = 1;
			aftCompensation = forwardDriveProportion;
		}
    	else if(driveMode == BALANCEDDRIVE) {
    		frontCompensation = balancedDriveProportion;
    		aftCompensation = balancedDriveProportion;
    	}
    	else{
    		frontCompensation = aftDriveProportion;
    		aftCompensation = 1;
    	}
    	setForwardAftCompensations();
    }
    
    private void setForwardAftCompensations() {
    	leftFrontMec.setCompensation(frontCompensation);
		rightFrontMec.setCompensation(frontCompensation);
		leftRearMec.setCompensation(aftCompensation);
		rightRearMec.setCompensation(aftCompensation);
    }
    
    private double getEncoderSpeed(Encoder encoder) {
    	long elapseTime = getElapsetime();
    	reset();
    	return encoder.getDistance() / elapseTime;
    }
    
    private long reset() {
    	return startTime = 0;
    	
    }
    
    private long getElapsetime() {
    	return startTime = System.currentTimeMillis() - startTime;
    }
    
    private void delay(int delayTime) {
		reset(); 
		
		while (true) {
			if (getElapsetime() > delayTime) {
				return;
			}
		}
	}

	/*
     * Used as a proof of concept for the I2C rangefinder 
     */
    public void displayRangeFinderDistance() {
    	if (!forwardRangeFinder.addressOnly()) {
    		forwardRangeFinder.write(204, 81);
    		if (getElapsetime() >= 0 && getElapsetime() < 3) {
				reset();
			}
    		
    		if (getElapsetime() > 80) {
    			forwardRangeFinder.read(225, 2, buffer);
    			
    			/*
    	    	 * Since we currently don't know what the hell this buffer array is, I will print everything from it!
    	    	 */

    			for (int i = 0; i < buffer.length; i++) {
    				SmartDashboard.putNumber("Buffer[ " + i + " ]", buffer[i]);
    			}
			}
		}
    	
    }
    
    
    

	/*private void circularJoystick(Joystick driveJoystick) {
    	double joystickX = driveJoystick.getX();
    	double joystickY = driveJoystick.getY();
    	double magnitude = driveJoystick.getMagnitude();
    	double angle = driveJoystick.getDirectionRadians();
    	if (magnitude > 1) {
			circularXVal = Math.cos(angle);
			circularYVal = Math.sin(angle);
			return;
		}
    	circularXVal = joystickX;
    	circularYVal = -joystickY;
    }
    */
    
   /*public void testMecanumDriveTopRight() {
	   leftFrontMec.testSet(.1, .2, 1.0, "diagonal top right left front", .3);
	   leftFrontMec.set(.1, .2, 1.0);
	   rightFrontMec.testSet(.1, .2, 1.0, "diagnoal top right right front", .1);
	   rightFrontMec.set(.1, .2, 1.0);
	   Joystick driveJoystick = Robot.oi.driveStick;
	   displayDriveInfo(driveJoystick);
   }
   */
    
    /*public void autoDriveForwardWithSensorAndDistance(double forwardDistance, boolean usingPortSensor) {
    	
    }
    */
    
    public void center(Ultrasonic rangefinder, Gyro gyro, double targetDistance, double targetHeading) {
    	double gyroAngle = gyro.getAngle();
    	if (gyroAngle <= targetHeading + 10 && gyroAngle >= targetHeading - 10) {
			
		}
    }
    
  //  private void findAngle() {
    	
  }


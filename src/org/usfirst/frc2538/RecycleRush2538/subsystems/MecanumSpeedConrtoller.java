package org.usfirst.frc2538.RecycleRush2538.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MecanumSpeedConrtoller {
	
	double xPull;
	double yPull;
	double compensation;
	boolean inverted;
	SpeedController talon;
	
	public MecanumSpeedConrtoller(SpeedController talon, double xPull, double yPull, double compensation, boolean inverted) {
		this.xPull = xPull;
		this.yPull = yPull;
		this.compensation = compensation;
		this.inverted = inverted;
		this.talon = talon;
	}
	
	public void setCompensation(double newComp) {
		if(inverted) {
			compensation = -newComp;
		}
		else{
			compensation = newComp;
		}
	}
	
	public double dotProduct(double xVal, double yVal, double throttle) {
		double xValThrottled = xVal * throttle;
		double yValThrottled = yVal * throttle;
		return (xPull * xValThrottled) + (yPull * yValThrottled);
	}
	
	public void set(double xVal, double yVal, double throttle) {
		talon.set(compensation * dotProduct(xVal, yVal, throttle));
	}
	
	public void spinSet(double joystickZ) {
		talon.set(joystickZ);
	}
	
	public void testSet(double xVal, double yVal, double throttle, String testName, double expectedValue) {
		if (compensation * dotProduct(xVal, yVal, throttle) == expectedValue) {
			SmartDashboard.putBoolean(testName, true);
		}
		SmartDashboard.putString(testName, "failed: " + (compensation * dotProduct(xVal, yVal, throttle)));
	}
}

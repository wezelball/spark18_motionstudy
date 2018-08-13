package org.usfirst.frc.team384.robot;

import edu.wpi.first.wpilibj.Timer;

public class MotionTest {

	//	Use a test timer for motion testing
	private Timer motionTestTimer = new Timer();
	
	// I hope this friggin works!
	private Drivetrain drivetrain = new Drivetrain(); 
	
	private boolean isFirstTime = true;
	public boolean isTestRunning;
	
	double enc_position, distance, velocity, accel, jerk;
	
	// Constructor
	public MotionTest()	{
		
		// Set up our custom logger.
		try {
			Logging.CustomLogger.setup();
		} catch (Throwable e) {
			Logging.logException(e);
		}
	}
	
	public void start()	{
		if(isFirstTime) {
			motionTestTimer.start();
			isTestRunning = true;
			
			// Clean up when you leave
			isFirstTime = false;
		} else	{
			if(!motionTestTimer.hasPeriodPassed(2.0))	{
				drivetrain.go(true, 0.5);
			} else	{
				drivetrain.stop();
				
				if (velocity < 0.1)	{	// robot
					isTestRunning = false;
				}
			}
			
			// Calculate data values, in olde kings units of kingsfoot/12 (inches), ye bastid
			enc_position = drivetrain.getEncoderPosition(Constants.DRIVE_LEFT);
			// Distance in inches
			distance = Constants.kDRIVE_DIST_PER_PULSE * enc_position;
			// Velocity in inches per second
			velocity = drivetrain.getEncoderVelocity(0) * Math.PI * 0.078125;
			// Acceleration in inches/sec^2
			accel = drivetrain.getImuAccelY() * 386.1;
			
			// Log the data
			Logging.consoleLog
				("Log: " + enc_position + "," + distance + "," + velocity + "," + accel);
		}
	}
	
	public void stop()	{
		isTestRunning = false;
		isFirstTime = true;
	}
}

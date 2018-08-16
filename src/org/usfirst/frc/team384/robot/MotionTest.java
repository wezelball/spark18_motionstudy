package org.usfirst.frc.team384.robot;

import edu.wpi.first.wpilibj.Timer;

public class MotionTest {

	//	Use a test timer for motion testing
	private Timer motionTestTimer = new Timer();
	
	// I hope this friggin works!
	private Drivetrain drivetrain = new Drivetrain(); 
	
	private boolean isFirstTime;
	public boolean isRunning;
	public boolean isStopping;
	
	double l_counts, l_distance, l_velocity; //, l_accel;
	double r_counts, r_distance, r_velocity; // , r_accel;
	
	
	// Constructor
	public MotionTest()	{
		
		// Set up our custom logger.
		try {
			Logging.CustomLogger.setup();
		} catch (Throwable e) {
			Logging.logException(e);
		}
	
		isFirstTime = true;
		isRunning = false;
		isStopping = false;
	}
	
	public void start(double runtime, double runpower)	{
		if(isFirstTime) {
			motionTestTimer.start();
			isRunning = true;
			drivetrain.initializeEncoders();
			
			// Clean up when you leave
			isFirstTime = false;
		} else	{
			if(!motionTestTimer.hasPeriodPassed(runtime) && !isStopping)	{
				drivetrain.go(true, runpower);
			} else	{
				drivetrain.stop();
				isStopping = true;
				//System.out.println("Stop command issued, velocity");
				if (l_velocity < 0.1)	{
					stop();
					//System.out.println("Zero velocity reached");
				}
			}
			
			// Calculate data values, in olde kings units of kingsfoot/12 (inches), ye bastid
			l_counts = drivetrain.getEncoderPosition(Constants.DRIVE_LEFT);
			r_counts = drivetrain.getEncoderPosition(Constants.DRIVE_RIGHT); 
			
			// Distance in inches
			l_distance = Constants.kDRIVE_DIST_PER_PULSE * l_counts;
			r_distance = Constants.kDRIVE_DIST_PER_PULSE * r_counts;
			
			
			// Velocity in inches per second
			l_velocity = drivetrain.getEncoderVelocity(Constants.DRIVE_LEFT) * Math.PI * 0.078125;
			r_velocity = drivetrain.getEncoderVelocity(Constants.DRIVE_RIGHT) * Math.PI * 0.078125;
			
			// Acceleration in inches/sec^2
			//l_accel = r_accel = drivetrain.getImuAccelY() * 386.1;
			
			
			// Log the data
//			Logging.consoleLog
//				("," + l_counts + "," + r_counts + "," +  l_distance + "," + r_distance + "," + l_velocity + "," + r_velocity + "," + l_accel + "," + r_accel);
			Logging.consoleLog
				("," + l_counts + "," + r_counts + "," +  l_distance + "," + r_distance + "," + l_velocity + "," + r_velocity);
		}
	}
	
	public void stop()	{
		isRunning = false;
		isFirstTime = true;
		isStopping = false;
	}
}

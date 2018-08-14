/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team384.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

	// Subsystem classes
	public OI oi;
	public Drivetrain drivetrain;
	public MotionTest motionTest;

	double runTime  = 10.0;
	double runPower = 1.0;

	// Constructor
	public Robot() {
		oi = new OI();
		drivetrain = new Drivetrain();
		motionTest = new MotionTest();
	}
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		drivetrain.imuZeroYaw();
		drivetrain.initializeEncoders();
		drivetrain.setBrakeMode(true);
	}


	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		// STICK0 (DRIVER)
		if (oi.getButtonHeld(Constants.STICK0,1))	{
			drivetrain.arcadeDrive(oi.getAxis(Constants.STICK0, Constants.YAXIS), 
					-oi.getAxis(Constants.STICK0, Constants.XAXIS));
		}
		
		// Take off fast as you can and log data if button pressed,
		// stop immediately if released
		if (oi.getButtonPressed(Constants.STICK0,2))	{
			motionTest.start(runTime, runPower);
		} else if (oi.getButtonReleased(Constants.STICK0,2))	{
			motionTest.stop();
		}
		
		if (motionTest.isRunning)	{
			motionTest.start(runTime, runPower);
		}
		
		/*
		 * Let's do some useful stuff with dashboard
		 */
		SmartDashboard.putNumber("Left encoder position: ", drivetrain.getEncoderPosition(Constants.DRIVE_LEFT));
		SmartDashboard.putNumber("Right encoder position: ", drivetrain.getEncoderPosition(Constants.DRIVE_RIGHT));
		SmartDashboard.putNumber("Left side velocity: ", drivetrain.getEncoderVelocity(Constants.DRIVE_LEFT));
		SmartDashboard.putNumber("Right side velocity: ", drivetrain.getEncoderVelocity(Constants.DRIVE_RIGHT));
		SmartDashboard.putNumber("NavX heading: ", drivetrain.imuGetYaw());
		SmartDashboard.putNumber("NavX velocity X: ", drivetrain.getImuVelocityX());
		SmartDashboard.putNumber("NavX velocity Y: ", drivetrain.getImuVelocityY());
		//SmartDashboard.putNumber("NavX velocity Z: ", drivetrain.getImuVelocityY());
		SmartDashboard.putNumber("NavX acceleration X: ", drivetrain.getImuAccelX());
		SmartDashboard.putNumber("NavX acceleration Y: ", drivetrain.getImuAccelY());
		//SmartDashboard.putNumber("NavX acceleration Z: ", drivetrain.getImuAccelZ());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}

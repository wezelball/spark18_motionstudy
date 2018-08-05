/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team384.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

	// Constructor
	public Robot() {
		oi = new OI();
		drivetrain = new Drivetrain();

		// Set up our custom logger.
		try {
			Logging.CustomLogger.setup();
		} catch (Throwable e) {
			Logging.logException(e);
		}
	}
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//m_chooser.addDefault("Default Auto", kDefaultAuto);
		//m_chooser.addObject("My Auto", kCustomAuto);
		//SmartDashboard.putData("Auto choices", m_chooser);
		drivetrain.imuZeroYaw();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		//m_autoSelected = m_chooser.getSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector",
		// 		kDefaultAuto);
		//System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
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

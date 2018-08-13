package org.usfirst.frc.team384.robot;

public class Constants {
	// Joysticks
	public static final int STICK0 = 0;
	public static final int STICK1 = 1;
	public static final int XAXIS	= 0;
	public static final int YAXIS	= 1;
	public static final int ZAXIS	= 2;
	public static final int ZROTATE	= 3;	// only for Saitek, get rid of for student sticks
	
	// Drivetrain
	public static final int DRIVE_LEFT = 0;
	public static final int DRIVE_RIGHT = 1;
	public static final int kDRIVE_ENC_PPR  = 512;	// Talon counts 4 edges * 128 PPR
	// Reducing wheel diameter make robot go farther for given drive distance
	public static final double kDRIVE_DIST_PER_PULSE = (4.0 * Math.PI) / kDRIVE_ENC_PPR;	// units are in inches	
	
	/*
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/*
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 10;

	/* choose so that Talon does not report sensor out of phase */
	public static final boolean kLeftSensorPhase = false;
	public static final boolean kRightSensorPhase = true;

	/* choose based on what direction you want to be positive,
		this does not affect motor invert. */
	public static final boolean kLeftMotorInvert = true;
	public static final boolean kRightMotorInvert = true;
}

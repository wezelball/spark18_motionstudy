package org.usfirst.frc.team384.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain {
	/* Instantiate talon motors */
	private WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(12);
	private WPI_TalonSRX rearRightMotor = new WPI_TalonSRX(8);
	private WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(6);		// -1.0 to run forward
	private WPI_TalonSRX rearLeftMotor = new WPI_TalonSRX(2);		// -1.0 to run forward

	// Define the motors that are slaved as a control group
	private SpeedControllerGroup leftDrivetrain = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
	private SpeedControllerGroup rightDrivetrain = new SpeedControllerGroup(frontRightMotor, rearRightMotor);
	
	// Set up differential drive for teleop
	private DifferentialDrive diffDrive = new DifferentialDrive(leftDrivetrain, rightDrivetrain);
	
	private AHRS imu;						// the NavX board

	// Drivetrain constructor
	public Drivetrain()	{
		// Initialize the IMU
		try {
			imu = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
			
		}
		
		/*
		 * Configure the Talon SRX motor controllers
		 * 
		 */
		frontLeftMotor.configSelectedFeedbackSensor(
				FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		frontRightMotor.configSelectedFeedbackSensor(
				FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		/* choose to ensure sensor is positive when output is positive */
		frontLeftMotor.setSensorPhase(Constants.kLeftSensorPhase);
		frontRightMotor.setSensorPhase(Constants.kRightSensorPhase);

		/* choose based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. */ 
		frontLeftMotor.setInverted(Constants.kLeftMotorInvert);
		frontRightMotor.setInverted(Constants.kRightMotorInvert);
		
		/* set the peak and nominal outputs, 12V means full */
		frontLeftMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		frontRightMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		
		frontLeftMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		frontRightMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		
		frontLeftMotor.configPeakOutputForward(1, Constants.kTimeoutMs);	// debug
		frontRightMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		
		frontLeftMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);	// debug
		frontRightMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
	
		rearLeftMotor.follow(frontLeftMotor); 	// follow the master talon
		rearLeftMotor.setInverted(true);		// runs backwards from front motor
		
		rearRightMotor.setInverted(true);		// runs backwards from front motor		
		rearRightMotor.follow(frontRightMotor);
	}
	
	/*
	 * Initialize encoders - set to zero
	 */
	public void initializeEncoders()	{
		frontLeftMotor.getSensorCollection().setQuadraturePosition(0, 0);
		frontRightMotor.getSensorCollection().setQuadraturePosition(0, 0);
	}

	/*
	 * Get encoder position
	 * 0 = left side drive rail
	 * 1 = right side drive rail
	 */
	public int getEncoderPosition(int side)	{
		int result;
		
		if(side == 0)	{
			result = frontLeftMotor.getSelectedSensorPosition(0);
		} else if (side == 1)	{
			result = frontRightMotor.getSelectedSensorPosition(0);
		}	else	{	// an illegal value was sent
			result = 0;
		}
		
		return result;
	}

	/*
	 * Get the encoder position as a double, averaging both sides
	 * This returns the distance in inches.
	 * Make sure that both encoder values are of same polarity
	 */
	public double getEncoderDistance()	{
		int leftEncoderDistance;
		int rightEncoderDistance;
		
		leftEncoderDistance = -frontLeftMotor.getSelectedSensorPosition(0);	// negative when forward
		rightEncoderDistance = frontRightMotor.getSelectedSensorPosition(0);
	
		// Return only the left encoder until the right encoder gets fixed
		return leftEncoderDistance * Constants.kDRIVE_DIST_PER_PULSE;
	}
	
	
	/*
	 * Get encoder velocity
	 * 0 = left side drive rail
	 * 1 = right side drive rail
	 * 
	 * Talon reports velocity is sensor units per 100ms. 
	 * Current sensor 128 PPR * 4 = 512 counts per revolution
	 */
	public int getEncoderVelocity(int side)	{
		int result;
		
		if(side == 0)	{
			result = frontLeftMotor.getSelectedSensorVelocity(0);
		} else if (side == 1)	{
			result = frontRightMotor.getSelectedSensorVelocity(0);
		}	else	{	// an illegal value was sent
			result = 0;
		}
		
		return result;
	}
	
	public void arcadeDrive(double speedaxis,  double turnaxis)	{
		diffDrive.arcadeDrive(speedaxis, turnaxis);
	}

	// Stop the motors	
	public void stop()	{
		frontLeftMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
		frontRightMotor.set(ControlMode.PercentOutput, 0.0);
	}
	
	// Run full speed
	// set fwd to true to go forward 
	public void go(boolean fwd, double speed)	{
		if(fwd)	{
			frontLeftMotor.set(ControlMode.PercentOutput, -speed);	
			frontRightMotor.set(ControlMode.PercentOutput, speed);
		} else	{
			frontLeftMotor.set(ControlMode.PercentOutput, speed);	
			frontRightMotor.set(ControlMode.PercentOutput, -speed);
		}
	}
	
	public void setBrakeMode(boolean brakeon)	{
		if (brakeon)	{
			frontRightMotor.setNeutralMode(NeutralMode.Brake);
			frontLeftMotor.setNeutralMode(NeutralMode.Brake);
			rearRightMotor.setNeutralMode(NeutralMode.Brake);
			frontLeftMotor.setNeutralMode(NeutralMode.Brake);
		}	else {
			frontRightMotor.setNeutralMode(NeutralMode.Coast);
			frontLeftMotor.setNeutralMode(NeutralMode.Coast);
			rearRightMotor.setNeutralMode(NeutralMode.Coast);
			frontLeftMotor.setNeutralMode(NeutralMode.Coast);
		}
	}
	
	public float imuGetYaw()	{
		return imu.getYaw();
	}
	
	public void imuZeroYaw()	{
		imu.zeroYaw();
	}

	// X axis runs side to side of robot
	public double getImuVelocityX()	{
		return imu.getVelocityX();
	}
	

	// Y axis runs front/rear of robot
	public double getImuVelocityY()	{
		return imu.getVelocityY();
	}
	
	// TODO - test me
	// Z axis runs up to ceiling/down to floor
	public double getImuVelocityZ()	{
		return imu.getVelocityZ();
	}
	
	/*
	 * The naVX returns acceleration in g's, adjusted for gravity
	 * To convert to inches/sec^2, multiply by 386.1
	 * To convert to feet/sec^2, multiply by 32.17
	 * To convert to meters/sec^2, multiply by 9.807
	 */
	
	// X axis runs side to side of robot
	// Returns acceleration in g's
	public double getImuAccelX()	{
		return imu.getWorldLinearAccelX();
	}
	
	// Y axis runs front/rear of robot
	// Returns acceleration in g's
	public double getImuAccelY()	{
		return imu.getWorldLinearAccelY();
	}
	
	// Returns acceleration in g's
	public double getImuAccelZ()	{
		return imu.getWorldLinearAccelZ();
	}
}

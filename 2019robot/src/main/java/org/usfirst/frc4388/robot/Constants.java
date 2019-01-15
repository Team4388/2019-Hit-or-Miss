
package org.usfirst.frc4388.robot;


/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */

public class Constants {

    public static double kDriveWheelDiameterInches = 6.04; 
    public static double kTrackLengthInches = 10;
    public static double kTrackWidthInches = 26.5;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
    public static double kTrackScrubFactor = 0.75;

    // Drive constants
    public static double kDriveLowGearMaxSpeedInchesPerSec = 12.0 * 5.0;
    public static double kDriveStraightBasicMaxSpeedInchesPerSec = 72.0;
    public static double kDriveStraightBasicMinSpeedInchesPerSec = 5.0;
    public static double kDriveStraightBasicYawErrorDivisor = 20.0;		// steer parameter will be -yawError divided by this
    public static double kDriveStraightBasicMaxSteerMagnitude = 0.8;	// keep absolute value of steer parameter below this
    public static double kDriveTurnBasicTankMotorOutput = 0.4;
    public static double kDriveTurnBasicSingleMotorOutput = 0.15;
    public static double kElevatorWheelDiameterInches = 1;
	// Encoders
	public static int kDriveEncoderTicksPerRev = 4096;
	public static double kDriveEncoderTicksPerInch = (double)kDriveEncoderTicksPerRev / (kDriveWheelDiameterInches * Math.PI);
	
	// Elevator
	public static int kElevatorEncoderTickesPerRev = 256;
    public static double kElevatorInchesOfTravelPerRev = 3.75;
	public static double kElevatorEncoderTicksPerInch = 126.36;
	public static double kElevatorBasicPercentOutputUp = -0.85;
	public static double kElevatorBasicPercentOutputDown =.7;
	
    // CONTROL LOOP GAINS

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 1.0;
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 6.0;
    public static double kDriveVelocityKf = 0.5;
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int kDriveVelocityAllowableError = 0;

    // PID gains for drive base lock loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int kDriveBaseLockAllowableError = 10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp = 4.0; // 6.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 50.0;

    // Path following constants
    public static double kPathFollowingLookahead = 24.0; // inches
    public static double kPathFollowingMaxVel = 120.0; // inches/sec
    public static double kPathFollowingMaxAccel = 80.0; // inches/sec^2

}

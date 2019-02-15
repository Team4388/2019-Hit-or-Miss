
package org.usfirst.frc4388.robot;


/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */

public class Constants {

    public static double kLooperDt = 0.01;
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
    public static double kArmWheelDiameterInches = 1;    //Changes depending on mechanical design

	// Encoders
	public static int kDriveEncoderTicksPerRev = 4096;
	public static double kDriveEncoderTicksPerInch = (double)kDriveEncoderTicksPerRev / (kDriveWheelDiameterInches * Math.PI);
	
	// Arm
	public static int kArmEncoderTickesPerRev = 4096;
    public static double kArmDegreesOfTravelPerRev = 360;
	public static double kArmEncoderTicksPerDegree = 11.38;
	public static double kArmBasicPercentOutputUp = -0.85;
	public static double kArmBasicPercentOutputDown =.7;
    

    
    //Wrist
    public static int kWristEncoderticksPerRev = 4096;
    public static double kWristDegreesOfTravel = 360;
    public static double kWristEncoderTicksPerDegree = 11.38;
















    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    
    // Drive constants

    // Geometry
    public static double kCenterToFrontBumperDistance = 16.33;
    public static double kCenterToIntakeDistance = 23.11;
    public static double kCenterToRearBumperDistance = 16.99;
    public static double kCenterToSideBumperDistance = 17.225;

    /* CONTROL LOOP GAINS */

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 0.2; //1.0;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 0.04;//0.04;
    public static double kDriveHighGearVelocityKf = 0.07;//0.07;
    public static int kDriveHighGearVelocityIZone = 200;
    public static double kDriveHighGearVelocityRampRate = 0.05; // 0.02
    public static double kDriveHighGearNominalOutput = 0.5/12.0;
    public static double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveLowGearVelocityKp = 0.1; //.1
    public static double kDriveLowGearVelocityKi = 0.0;
    public static double kDriveLowGearVelocityKd = 0.04; // .04
    public static double kDriveLowGearVelocityKf = 0.03;  // .07
    public static int kDriveLowGearVelocityIZone = 200;
    public static double kDriveLowGearVelocityRampRate = 0.02;
    public static double kDriveLowGearNominalOutput = 0.1/12.0;
    public static double kDriveLowGearMaxSetpoint = 10.0 * 12.0; // 8 fps

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveLowGearPositionKp = 1.0;  // 1.0
    public static double kDriveLowGearPositionKi = 0.002;   //0.002
    public static double kDriveLowGearPositionKd = 0.04;
    public static double kDriveLowGearPositionKf = .45; // 0.45
    public static int kDriveLowGearPositionIZone = 700;
    public static double kDriveLowGearPositionRampRate = 0.05; // V/s
    public static double kDriveLowGearMaxVelocity = 8.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 8 fps
                                                                                                               // in RPM
    public static double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
                                                                                                             // in RPM/s

    public static double kDriveVoltageCompensationRampRate = 0.0;

    // Turn to heading gains
    public static double kDriveTurnKp = 3.0;
    public static double kDriveTurnKi = 1.5;
    public static double kDriveTurnKv = 0.0;
    public static double kDriveTurnKffv = 1.0;
    public static double kDriveTurnKffa = 0.0;
    public static double kDriveTurnMaxVel = 360.0;
    public static double kDriveTurnMaxAcc = 720.0;

    // Path following constants
    public static double kMinLookAhead = 12.0; // inches
    public static double kMinLookAheadSpeed = 9.0; // inches per second
    public static double kMaxLookAhead = 24.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
//    public static double kSegmentCompletionTolerance = 0.1; // inches
//    public static double kPathFollowingMaxAccel = 120.0; // inches per second^2
//    public static double kPathFollowingMaxVel = 120.0; // inches per second
//    public static double kPathFollowingProfileKp = 0.05;  //5.0
//    public static double kPathFollowingProfileKi = 0.03;
//    public static double kPathFollowingProfileKv = 0.02;
//    public static double kPathFollowingProfileKffv = 1.0;
//    public static double kPathFollowingProfileKffa = 0.05;
//    public static double kPathFollowingGoalPosTolerance = 0.75;
//    public static double kPathFollowingGoalVelTolerance = 12.0;
//    public static double kPathStopSteeringDistance = 9.0;

    public static double kSegmentCompletionTolerance = 0.1; // inches
    public static double kPathFollowingMaxAccel = 90.0; // inches per second^2
    public static double kPathFollowingMaxVel = 150.0; // inches per second
    public static double kPathFollowingProfileKp = 5.0;  //5.0
    public static double kPathFollowingProfileKi = 0.03;  // 0.03
    public static double kPathFollowingProfileKv = 0.02; //0.02
    public static double kPathFollowingProfileKffv = 1.2;  //1.2
    public static double kPathFollowingProfileKffa = 0.05;  //0.05
    public static double kPathFollowingGoalPosTolerance = 0.75;
    public static double kPathFollowingGoalVelTolerance = 12.0;
    public static double kPathStopSteeringDistance = 9.0;




















	
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


}

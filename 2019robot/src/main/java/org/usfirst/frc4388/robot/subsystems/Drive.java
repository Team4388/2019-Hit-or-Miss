
package org.usfirst.frc4388.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.sensors.PigeonIMU;
//import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.OI;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.utility.BHRDifferentialDrive;
import org.usfirst.frc4388.utility.BHRMathUtils;
import org.usfirst.frc4388.utility.Loop;
import org.usfirst.frc4388.utility.MPSoftwarePIDController;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import org.usfirst.frc4388.utility.MPTalonPIDController;
import org.usfirst.frc4388.utility.PIDParams;
import org.usfirst.frc4388.utility.SoftwarePIDController;
import org.usfirst.frc4388.utility.TalonSRXEncoder;
import org.usfirst.frc4388.utility.TalonSRXFactory;
import org.usfirst.frc4388.utility.control.Kinematics;
import org.usfirst.frc4388.utility.control.Lookahead;
import org.usfirst.frc4388.utility.control.Path;
import org.usfirst.frc4388.utility.control.PathFollower;
import org.usfirst.frc4388.utility.control.RobotState;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.math.Rotation2d;
import org.usfirst.frc4388.utility.math.Twist2d;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
/*
public class Drive extends Subsystem implements Loop {
	private static Drive instance;

	public static enum DriveControlMode {
		JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, ADAPTIVE_PURSUIT, VELOCITY_SETPOINT, CAMERA_TRACK
	};

	public static enum DriveSpeedShiftState { HI, LO};

	public static enum ClimberState {
		DEPLOYED, RETRACTED
	};

	// One revolution of the wheel = Pi * D inches = 60/24 revs due to gears * 36/12
	// revs due mag encoder gear on ball shifter * 4096 ticks
	public static final double ENCODER_TICKS_TO_INCHES = (36.0 / 12.0) * (60.0 / 24.0) * 4096.0 / (5.8 * Math.PI);
	public static final double TRACK_WIDTH_INCHES = 24.56; // 26.937;

	// Motion profile max velocities and accel times
	public static final double MAX_TURN_RATE_DEG_PER_SEC = 320;
	public static final double MP_AUTON_MAX_STRAIGHT_VELOCITY_INCHES_PER_SEC = 140; // 72;
	public static final double MP_AUTON_MAX_LO_GEAR_STRAIGHT_VELOCITY_INCHES_PER_SEC = 320;
	public static final double MP_AUTON_MAX_HIGH_GEAR_STRAIGHT_VELOCITY_INCHES_PER_SEC = 400;
	public static final double MP_AUTON_MAX_TURN_RATE_DEG_PER_SEC = 270;
	public static final double MP_SLOW_VELOCITY_INCHES_PER_SEC = 25;
	public static final double MP_SLOW_MEDIUM_VELOCITY_INCHES_PER_SEC = 50;
	public static final double MP_MEDIUM_VELOCITY_INCHES_PER_SEC = 80;
	public static final double MP_FAST_VELOCITY_INCHES_PER_SEC = 100;

	public static final double MP_STRAIGHT_T1 = 600;
	public static final double MP_STRAIGHT_T2 = 300;
	public static final double MP_TURN_T1 = 600;
	public static final double MP_TURN_T2 = 300;
	public static final double MP_MAX_TURN_T1 = 400;
	public static final double MP_MAX_TURN_T2 = 200;
	
	public static final double OPEN_LOOP_VOLTAGE_RAMP_HI = 0.0;
	public static final double OPEN_LOOP_VOLTAGE_RAMP_LO = 0.1;
	
	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();	

	private TalonSRXEncoder leftDrive1;
	private TalonSRX leftDrive2;
	private TalonSRX leftDrive3;

	private TalonSRXEncoder rightDrive1;
	private TalonSRX rightDrive2;
	private TalonSRX rightDrive3;

	private BHRDifferentialDrive m_drive;
	
	private boolean isRed = true;
    private boolean mIsBrakeMode;
	
	private long periodMs = (long)(Constants.kLooperDt * 1000.0);
	
    protected Rotation2d mAngleAdjustment = Rotation2d.identity();

	// Pneumatics
	private Solenoid speedShift;
	private DriveSpeedShiftState shiftState = DriveSpeedShiftState.HI;

	// Input devices
	public static final int DRIVER_INPUT_JOYSTICK_ARCADE = 0;
	public static final int DRIVER_INPUT_JOYSTICK_TANK = 1;
	public static final int DRIVER_INPUT_JOYSTICK_CHEESY = 2;
	public static final int DRIVER_INPUT_XBOX_CHEESY = 3;
	public static final int DRIVER_INPUT_XBOX_ARCADE_LEFT = 4;
	public static final int DRIVER_INPUT_XBOX_ARCADE_RIGHT = 5;
	public static final int DRIVER_INPUT_WHEEL = 6;

	public static final double STEER_NON_LINEARITY = 0.5;
	public static final double MOVE_NON_LINEARITY = 1.0;
	
	public static final double STICK_DEADBAND = 0.02;
	
//	public static final double PITCH_THRESHOLD_1 = 20;
	public static final double PITCH_THRESHOLD_2 = 25;
	
	private int pitchWindowSize = 5;
	private int windowIndex = 0;
	private double pitchSum = 0;
	private double[] pitchAverageWindow = new double[pitchWindowSize];

	private int m_moveNonLinear = 0;
	private int m_steerNonLinear = -3;

	private double m_moveScale = 1.0;
	private double m_steerScale = 1.0;

	private double m_moveInput = 0.0;
	private double m_steerInput = 0.0;

	private double m_moveOutput = 0.0;
	private double m_steerOutput = 0.0;

	private double m_moveTrim = 0.0;
	private double m_steerTrim = 0.0;

	private boolean isFinished;
	private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;
	
    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
    private static final int kLowGearVelocityControlSlot = 2;

    private MPTalonPIDController mpStraightController;
//	private PIDParams mpStraightPIDParams = new PIDParams(0.1, 0, 0, 0.005, 0.03, 0.15);  // 4 colsons
	private PIDParams mpStraightPIDParams = new PIDParams(0.05, 0, 0, 0.0008, 0.004, 0.03);  // 4 omni
	private PIDParams mpHoldPIDParams = new PIDParams(1, 0, 0, 0.0, 0.0, 0.0); 

	private MPSoftwarePIDController mpTurnController; // p    i   d     a      v      g    izone
//	private PIDParams mpTurnPIDParams = new PIDParams(0.07, 0.00002, 0.5, 0.00025, 0.008, 0.0, 100);  // 4 colson wheels
	private PIDParams mpTurnPIDParams = new PIDParams(0.03, 0.00002, 0.4, 0.0004, 0.0030, 0.0, 100);  // 4 omni
	
	private SoftwarePIDController pidTurnController;
	private PIDParams pidTurnPIDParams = new PIDParams(0.04, 0.001, 0.4, 0, 0, 0.0, 100); //i=0.0008

//	private PIDParams adaptivePursuitPIDParams = new PIDParams(0.1, 0.00, 1, 0.44); 
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;
    private RobotState mRobotState = RobotState.getInstance();
	
	//private PigeonIMU gyroPigeon;
	//private double[] yprPigeon = new double[3];
	//private boolean useGyroLock;
	//private double gyroLockAngleDeg;
	// double kPGyro = 0.04;
	//private boolean isCalibrating = false;
	//private double gyroOffsetDeg = 0;

	private AHRS gyroNavX;
	private boolean useGyroLock;
	private double gyroLockAngleDeg;
	//private double kPGyro = 0.04;
	private double kPGyro = 0.0625;
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;
	
	private double mLastValidGyroAngle;
	private double mCameraVelocity;
	private double kCamera = 0.8;
	
	private double limeArea;
	private double limeX;
	private double limeY;
	private double limeSkew;
	private boolean isLimeValid;
	private double LEDMode;
	private double camMode;
	/*
    /**
     * Check if the drive talons are configured for velocity control
     ////////////////////////////////////////
    protected static boolean usesTalonVelocityControl(DriveControlMode state) {
        if (state == DriveControlMode.VELOCITY_SETPOINT || state == DriveControlMode.ADAPTIVE_PURSUIT || state == DriveControlMode.CAMERA_TRACK) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     /////////////////////////////////////////////////////
    protected static boolean usesTalonPositionControl(DriveControlMode state) {
        if (state == DriveControlMode.MP_STRAIGHT ||
                state == DriveControlMode.MP_TURN ||
                state == DriveControlMode.HOLD) {
            return true;
        }
        return false;
    }

    private Drive() {
		try {
			leftDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, false, FeedbackDevice.QuadEncoder);
			leftDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID, RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			

			rightDrive1 = TalonSRXFactory.createTalonEncoder(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID, ENCODER_TICKS_TO_INCHES, true, FeedbackDevice.QuadEncoder);
			rightDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID, RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			
			
			leftDrive1.setSafetyEnabled(false);
			leftDrive1.setSensorPhase(false);  
			
			leftDrive1.setInverted(true);
			leftDrive2.setInverted(true);
			leftDrive3.setInverted(true);
			
			rightDrive1.setSafetyEnabled(false);
			rightDrive1.setSensorPhase(false);  
			
			rightDrive1.setInverted(false);			
			rightDrive2.setInverted(false);
			rightDrive3.setInverted(false);
							
			motorControllers.add(leftDrive1);
			motorControllers.add(rightDrive1);
			
			m_drive = new BHRDifferentialDrive(leftDrive1, rightDrive1);
			m_drive.setSafetyEnabled(false);


			System.out.println("this should be first");
			gyroNavX = new AHRS(SPI.Port.kMXP);
//			gyroPigeon.clearStickyFaults(10);
			
			speedShift = new Solenoid(RobotMap.DRIVETRAIN_SPEEDSHIFT_PCM_ID);
									
			loadGains();
        	setBrakeMode(true);
	}
		catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}
    
    private void setOpenLoopVoltageRamp(double timeTo12VSec) {
		leftDrive1.configOpenloopRamp(timeTo12VSec, TalonSRXEncoder.TIMEOUT_MS);
		rightDrive1.configOpenloopRamp(timeTo12VSec, TalonSRXEncoder.TIMEOUT_MS);
    }

    public synchronized void loadGains() {
        leftDrive1.setPIDFIZone(kLowGearVelocityControlSlot, 
        		Constants.kDriveLowGearVelocityKp, 
        		Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, 
                Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone);
        
        rightDrive1.setPIDFIZone(kLowGearVelocityControlSlot, 
        		Constants.kDriveLowGearVelocityKp, 
        		Constants.kDriveLowGearVelocityKi,
                Constants.kDriveLowGearVelocityKd, 
                Constants.kDriveLowGearVelocityKf,
                Constants.kDriveLowGearVelocityIZone);
        
        leftDrive1.setPIDFIZone(kHighGearVelocityControlSlot, 
        		Constants.kDriveHighGearVelocityKp, 
        		Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, 
                Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone);
        
        rightDrive1.setPIDFIZone(kHighGearVelocityControlSlot, 
        		Constants.kDriveHighGearVelocityKp, 
        		Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, 
                Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone);        
    }

    @Override
	public void initDefaultCommand() {
	}
	
	public double getGyroAngleDeg() {
		//return getGyroPigeonAngleDeg();
		return getGyroNavXAngleDeg();
	}
	
	public double getGyroNavXAngleDeg() {
		return gyroNavX.getAngle() + gyroOffsetDeg;
	}
	/*
	public synchronized double getGyroPitchAngle() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return  yprPigeon[2];
	}



///////////////////////////////////////////////// /*
	public boolean checkPitchAngle() {
		double pitchAngle = Math.abs(getGyroPitchAngle());
		if(pitchAngle > 10) {
			return true;
		}
		return false;
	}

	///////////////////////////////////////////////////////////// * /
	
	public synchronized void resetGyro() {
			//gyroPigeon.SetYaw(0);
			System.out.println("IF this works the screw the thing bellow");
			gyroNavX.zeroYaw();
	}
	
    public synchronized Rotation2d getGyroAngle() {
        return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(-getGyroAngleDeg()));
    }

    public synchronized void setGyroAngle(Rotation2d adjustment) {
    	resetGyro();
        mAngleAdjustment = adjustment;
    }

	public synchronized void resetEncoders() {
		rightDrive1.setPosition(0);
		leftDrive1.setPosition(0);
	}
	
    public void zeroSensors() {
        resetEncoders();
        resetGyro();
    }

    public void calibrateGyro() {
		//gyroPigeon.enterCalibrationMode(CalibrationMode.Temperature, TalonSRXEncoder.TIMEOUT_MS);
	}
	
	public void endGyroCalibration() {
		if (isCalibrating == true) {
			isCalibrating = false;
		}
	}
	
	public void setGyroOffset(double offsetDeg) {
		gyroOffsetDeg = offsetDeg;
	}
		
	public void setStraightMP(double distanceInches, double maxVelocity, boolean useGyroLock, boolean useAbsolute, double desiredAbsoluteAngle) {
		double yawAngle = useAbsolute ? BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), desiredAbsoluteAngle) : getGyroAngleDeg();
		mpStraightController.setPID(mpStraightPIDParams, kLowGearPositionControlSlot);
		mpStraightController.setPIDSlot(kLowGearPositionControlSlot);
		mpStraightController.setMPStraightTarget(0, distanceInches, maxVelocity, MP_STRAIGHT_T1, MP_STRAIGHT_T2, useGyroLock, yawAngle, true); 
		setControlMode(DriveControlMode.MP_STRAIGHT);
	}
		
	public void setRelativeTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec, MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
		
	public void setRelativeMaxTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec, MP_MAX_TURN_T1, MP_MAX_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
	
	public void setAbsoluteTurnMP(double absoluteTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		mpTurnController.setMPTurnTarget(getGyroAngleDeg(), BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), absoluteTurnAngleDeg), turnRateDegPerSec, MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		setControlMode(DriveControlMode.MP_TURN);
	}
	
    public void setDriveHold(boolean status) {
		if (status) {
			setControlMode(DriveControlMode.HOLD);
		}
		else {
			setControlMode(DriveControlMode.JOYSTICK);
		}
	}
    
    public synchronized void setControlMode(DriveControlMode controlMode) {
 		this.driveControlMode = controlMode;
 		if (controlMode == DriveControlMode.HOLD) {
			mpStraightController.setPID(mpHoldPIDParams, kLowGearPositionControlSlot);
			leftDrive1.setPosition(0);
			leftDrive1.set(ControlMode.Position, 0);
			rightDrive1.setPosition(0);
			rightDrive1.set(ControlMode.Position, 0);
		}
		setFinished(false);
	}
    
    public synchronized DriveControlMode getControlMode() {
    	return driveControlMode;
    }
	
	@Override
	public void onStart(double timestamp) {
        synchronized (Drive.this) {
			mpStraightController = new MPTalonPIDController(periodMs, motorControllers);
			mpStraightController.setPID(mpStraightPIDParams, kLowGearPositionControlSlot);
			mpTurnController = new MPSoftwarePIDController(periodMs, mpTurnPIDParams, motorControllers);
			pidTurnController = new SoftwarePIDController(pidTurnPIDParams, motorControllers);
        }
	}

	@Override
	public void onStop(double timestamp) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (Drive.this) {
			DriveControlMode currentControlMode = getControlMode();

			if (currentControlMode == DriveControlMode.JOYSTICK) {
				driveWithJoystick();
			}
			else if (!isFinished()) {
				switch (currentControlMode) {			
					case MP_STRAIGHT :
						setFinished(mpStraightController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case MP_TURN:
						setFinished(mpTurnController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case PID_TURN:
						setFinished(pidTurnController.controlLoopUpdate(getGyroAngleDeg())); 
	                    break;
					case ADAPTIVE_PURSUIT:
	                    if (mPathFollower != null) {
	                        updatePathFollower(timestamp);
	                    }
	                    return;
					case CAMERA_TRACK:
	                    updateCameraTrack();
	                    return;
	                default:
	                    System.out.println("Unknown drive control mode: " + currentControlMode);
	                    break;
                }
			}
			else {
				// hold in current state
			}
		}
	}
	
	public synchronized void setSpeed(double speed) {
		if (speed == 0) {
			setControlMode(DriveControlMode.JOYSTICK);
		}
		else {
			setControlMode(DriveControlMode.MANUAL);
			rightDrive1.set(ControlMode.PercentOutput, speed);
			leftDrive1.set(ControlMode.PercentOutput, speed);
		}
	}
	
	public synchronized void setGyroLock(boolean useGyroLock, boolean snapToAbsolute0or180) {
		if (snapToAbsolute0or180) {
			gyroLockAngleDeg = BHRMathUtils.adjustAccumAngleToClosest180(getGyroAngleDeg());
		}
		else {
			gyroLockAngleDeg = getGyroAngleDeg();
		}
		this.useGyroLock = useGyroLock;
	}

    /**
     * Called periodically when the robot is in cmaera track mode.
     //////////////////////////////////////////////////////////* /
    private void updateCameraTrack() {
    	updateLimelight();
    	double deltaVelocity = 0;
        if (isLimeValid) {
        	deltaVelocity = limeX * kCamera;
            mLastValidGyroAngle = getGyroAngleDeg();
            System.out.println("Valid lime angle = " + limeX);
        } else {
        	deltaVelocity = (getGyroAngleDeg() - mLastValidGyroAngle) * kCamera;
            System.out.println("In Valid lime angle = " + limeX);
        }
        updateVelocitySetpoint(mCameraVelocity + deltaVelocity, mCameraVelocity - deltaVelocity);
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     //////////////////////////////////////////////// * /
    public synchronized void setCameraTrack(double straightVelocity) {
        if (driveControlMode != DriveControlMode.CAMERA_TRACK) {
        	setFinished(false);
            configureTalonsForSpeedControl();
            driveControlMode = DriveControlMode.CAMERA_TRACK;
            mLastValidGyroAngle = getGyroAngleDeg();
            mCameraVelocity = straightVelocity;
        } else {
            setVelocitySetpoint(0, 0);
            System.out.println("Oh NOOOO in velocity set point for camera track");
        }
    }

    /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     ///////////////////////////////////////////////////////// * /
    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     * 
     * @see Path
     //////////////////////////////////////////////////////////////* /
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || driveControlMode != DriveControlMode.ADAPTIVE_PURSUIT) {
            configureTalonsForSpeedControl();
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));

            driveControlMode = DriveControlMode.ADAPTIVE_PURSUIT;
            mCurrentPath = path;
        } else {
            setVelocitySetpoint(0, 0);
            System.out.println("Oh NOOOO in velocity set point");
        }
    }

    /**
     * Start up velocity mode. This sets the drive train in high gear as well.
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     //////////////////////////////////////////////////* /
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlMode = DriveControlMode.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * Adjust Velocity setpoint (if already in velocity mode)
     * 
     * @param left_inches_per_sec
     * @param right_inches_per_sec
     ////////////////////////////////////////////////////////////////* /
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(driveControlMode)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double maxSetpoint = getShiftState() == DriveSpeedShiftState.HI ? Constants.kDriveHighGearMaxSetpoint : Constants.kDriveLowGearMaxSetpoint;
            final double scale = max_desired > maxSetpoint ? maxSetpoint / max_desired : 1.0;
            
            leftDrive1.setVelocityWorld(left_inches_per_sec * scale);
            rightDrive1.setVelocityWorld(right_inches_per_sec * scale);
//            double command = leftDrive1.convertEncoderWorldToTicks(left_inches_per_sec * scale) * 0.1;
//            System.out.println("vel Com u/s = " + command + ", vel com in/sec= " + left_inches_per_sec * scale + ", scale = " + scale + ", left pos in = " + getLeftPositionInches()  + ", right pos in = " + getRightPositionInches() + ", left vel in/sec = " + getLeftVelocityInchesPerSec() + ", left vel u/s = " + leftDrive1.getSelectedSensorVelocity(0));
        } else {
            System.out.println("Hit a bad velocity control state");
            leftDrive1.set(ControlMode.Velocity, 0);
            rightDrive1.set(ControlMode.Velocity, 0);
        }
    }

    /**
     * Configures talons for velocity control
     //////////////////////////////////////////////////////////////* /
    public void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(driveControlMode)) {
        	leftDrive1.enableVoltageCompensation(true);
        	leftDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
        	leftDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
        	leftDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);

        	rightDrive1.enableVoltageCompensation(true);
        	rightDrive1.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
        	rightDrive1.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
        	rightDrive1.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);
       	
        	if (getShiftState() == DriveSpeedShiftState.HI) {
        		System.out.println("configureTalonsForSpeedControl HI");
	        	leftDrive1.selectProfileSlot(kHighGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
	        	leftDrive1.configNominalOutputForward(Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	leftDrive1.configNominalOutputReverse(-Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	            leftDrive1.configClosedloopRamp(Constants.kDriveHighGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
	        	    	
	        	rightDrive1.selectProfileSlot(kHighGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
	        	rightDrive1.configNominalOutputForward(Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	rightDrive1.configNominalOutputReverse(-Constants.kDriveHighGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	rightDrive1.configClosedloopRamp(Constants.kDriveHighGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
        	}
        	else {
        		System.out.println("configureTalonsForSpeedControl LO");
	        	leftDrive1.selectProfileSlot(kLowGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
	        	leftDrive1.configNominalOutputForward(Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	leftDrive1.configNominalOutputReverse(-Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	            leftDrive1.configClosedloopRamp(Constants.kDriveLowGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
	        	    	
	        	rightDrive1.selectProfileSlot(kLowGearVelocityControlSlot, TalonSRXEncoder.PID_IDX);
	        	rightDrive1.configNominalOutputForward(Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	rightDrive1.configNominalOutputReverse(-Constants.kDriveLowGearNominalOutput, TalonSRXEncoder.TIMEOUT_MS);
	        	rightDrive1.configClosedloopRamp(Constants.kDriveLowGearVelocityRampRate, TalonSRXEncoder.TIMEOUT_MS);
        	}
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode 1");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode 2, control mode = " + driveControlMode);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (driveControlMode == DriveControlMode.ADAPTIVE_PURSUIT && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode 3. Control mode = " + driveControlMode);
            return false;
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            rightDrive1.setNeutralMode(NeutralMode.Brake);
            rightDrive2.setNeutralMode(NeutralMode.Brake);
            rightDrive3.setNeutralMode(NeutralMode.Brake);
            leftDrive1.setNeutralMode(NeutralMode.Brake);
            leftDrive2.setNeutralMode(NeutralMode.Brake);
            leftDrive3.setNeutralMode(NeutralMode.Brake);
        }
    }

    public synchronized void driveWithJoystick() {
		if(m_drive == null) return;

		m_moveInput = OI.getInstance().getDriverController().getLeftYAxis();
		m_steerInput = -OI.getInstance().getDriverController().getRightXAxis();
		
		m_moveOutput = adjustForSensitivity(m_moveScale, m_moveTrim,
					m_moveInput, m_moveNonLinear, MOVE_NON_LINEARITY);
		m_steerOutput = adjustForSensitivity(m_steerScale, m_steerTrim,
				m_steerInput, m_steerNonLinear, STEER_NON_LINEARITY);

		if (useGyroLock) {
			double yawError = gyroLockAngleDeg - getGyroAngleDeg();
			m_steerOutput = kPGyro * yawError;
		}
			/*	
		double pitchAngle = updatePitchWindow();
		if(Math.abs(pitchAngle) > PITCH_THRESHOLD_2) {
			m_moveOutput = Math.signum(pitchAngle) * -1.0;
			m_steerOutput = 0;
			System.out.println("Pitch Treshhold 2 angle = " + pitchAngle);
		}
/////////////////////////////////////////////////* /
		m_drive.arcadeDrive(-m_moveOutput, -m_steerOutput);	
	}
	
	


	/*
    private double updatePitchWindow() {
		double lastPitchAngle = pitchAverageWindow[windowIndex];
		double currentPitchAngle = getGyroPitchAngle();
		pitchAverageWindow[windowIndex] = currentPitchAngle;
		pitchSum = pitchSum - lastPitchAngle + currentPitchAngle;

		windowIndex++;
		if (windowIndex == pitchWindowSize) {
			windowIndex = 0;
		}	
		
    	return pitchSum/pitchWindowSize;
	}
	//////////////////////////////////////////////////////* /
    
	private boolean inDeadZone(double input) {
		boolean inDeadZone;
		if (Math.abs(input) < STICK_DEADBAND) {
			inDeadZone = true;
		} else {
			inDeadZone = false;
		}
		return inDeadZone;
	}

	public double adjustForSensitivity(double scale, double trim,
			double steer, int nonLinearFactor, double wheelNonLinearity) {
		if (inDeadZone(steer))
			return 0;

		steer += trim;
		steer *= scale;
		steer = limitValue(steer);

		int iterations = Math.abs(nonLinearFactor);
		for (int i = 0; i < iterations; i++) {
			if (nonLinearFactor > 0) {
				steer = nonlinearStickCalcPositive(steer, wheelNonLinearity);
			} else {
				steer = nonlinearStickCalcNegative(steer, wheelNonLinearity);
			}
		}
		return steer;
	}

	private double limitValue(double value) {
		if (value > 1.0) {
			value = 1.0;
		} else if (value < -1.0) {
			value = -1.0;
		}
		return value;
	}

	private double nonlinearStickCalcPositive(double steer,
			double steerNonLinearity) {
		return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer)
				/ Math.sin(Math.PI / 2.0 * steerNonLinearity);
	}

	private double nonlinearStickCalcNegative(double steer,
			double steerNonLinearity) {
		return Math.asin(steerNonLinearity * steer)
				/ Math.asin(steerNonLinearity);
	}

	public void setShiftState(DriveSpeedShiftState state) {
		shiftState = state;

		System.out.println("shift state = " + state);
		setOpenLoopVoltageRamp(state == DriveSpeedShiftState.HI ? OPEN_LOOP_VOLTAGE_RAMP_HI : OPEN_LOOP_VOLTAGE_RAMP_LO);
		if(state == DriveSpeedShiftState.HI) {
			speedShift.set(false);
		}
		else if(state == DriveSpeedShiftState.LO) {
			speedShift.set(true);
		}
	}

	public DriveSpeedShiftState getShiftState() {
		return shiftState;
	}

	public synchronized boolean isFinished() {
		return isFinished;
	}
	
	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}
	
	public double getPeriodMs() {
		return periodMs;
	}
	
	public boolean isRed() {
		return isRed;
	}
	
	public void setIsRed(boolean status) {
		isRed = status;
	}
	
	public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
    
    public double getRightPositionInches() {
    	return rightDrive1.getPositionWorld();
    }

    public double getLeftPositionInches() {
    	return leftDrive1.getPositionWorld();
    }

    public double getRightVelocityInchesPerSec() {
    	return rightDrive1.getVelocityWorld();
    }

    public double getLeftVelocityInchesPerSec() {
    	return leftDrive1.getVelocityWorld();
    }
    
    public double getAverageLeftCurrent() {
    	return (leftDrive1.getOutputCurrent() + leftDrive2.getOutputCurrent() + leftDrive3.getOutputCurrent()) / 3;
    }

    public double getAverageRightCurrent() {
    	return (rightDrive1.getOutputCurrent() + rightDrive2.getOutputCurrent() + rightDrive3.getOutputCurrent()) / 3;
    }
    
	public NetworkTable getLimetable() {
		return NetworkTableInstance.getDefault().getTable("limelight");
	}

	private void updateLimelight() {
		NetworkTable limeTable = getLimetable();
		
		double valid = limeTable.getEntry("tv").getDouble(0); 
		if (valid == 0) {
			isLimeValid = false;
		}
		else if (valid == 1) {
			isLimeValid = true;
		}
		
		limeX = limeTable.getEntry("tx").getDouble(0); 
		limeY = limeTable.getEntry("ty").getDouble(0); 
		limeArea = limeTable.getEntry("ta").getDouble(0); 
		limeSkew = limeTable.getEntry("ts").getDouble(0); 
	}
	
	//Set the LED mode of the limelight
	public void setLimeLED(boolean isOn) {
		getLimetable().getEntry("ledMode").setDouble(isOn ? 0 : 1);
	}
	
	//Set the camera mode
	public void setLimeCameraMode(boolean isOn) {
		getLimetable().getEntry("camMode").setDouble(isOn ? 1 : 0);
	}

	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Drive Right Position Inches", rightDrive1.getPositionWorld());
				SmartDashboard.putNumber("Drive Left Position Inches", leftDrive1.getPositionWorld());
				SmartDashboard.putNumber("Drive Right Velocity InPerSec", rightDrive1.getVelocityWorld());
				SmartDashboard.putNumber("Drive Left Velocity InPerSec", leftDrive1.getVelocityWorld());
				SmartDashboard.putNumber("Drive Left 1 Amps", leftDrive1.getOutputCurrent());
				SmartDashboard.putNumber("Drive Left 2 Amps", leftDrive2.getOutputCurrent());
				SmartDashboard.putNumber("Drive Left 3 Amps", leftDrive3.getOutputCurrent());
				SmartDashboard.putNumber("Drive Left Average Amps", getAverageLeftCurrent());
				SmartDashboard.putNumber("Drive Right 1 Amps", rightDrive1.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right 2 Amps", rightDrive2.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right 3 Amps", rightDrive3.getOutputCurrent());
				SmartDashboard.putNumber("Drive Right Average Amps", getAverageRightCurrent());
				SmartDashboard.putNumber("Yaw Angle Deg", getGyroAngleDeg());
				SmartDashboard.putData("Diff Drive", m_drive);
				NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
				NetworkTableEntry tx = table.getEntry("tx");
				NetworkTableEntry ty = table.getEntry("ty");
				NetworkTableEntry ta = table.getEntry("ta");
				SmartDashboard.putNumber("Limelight Valid", table.getEntry("tv").getDouble(0));
				SmartDashboard.putNumber("Limelight X", table.getEntry("tx").getDouble(0));
				SmartDashboard.putNumber("Limelight Y", table.getEntry("ty").getDouble(0));
				SmartDashboard.putNumber("Limelight Area", table.getEntry("ta").getDouble(0));
			}
			catch (Exception e) {
			}
		}
	}	
	
	public static Drive getInstance() {
		if(instance == null) {
			instance = new Drive();
		}
		return instance;
	}

}




















/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



import frc.robot.commands.DriveCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  
  static double LOW_GEAR_RATIO = 4.821;//6.57992
  static double HIGH_GEAR_RATIO = ;
  static double CIRCUMFERENCE = 15.221;
  double HIGH_INCHES_PER_REV = CIRCUMFERENCE/HIGH_GEAR_RATIO;
  double LOW_INCHES_PER_REV = CIRCUMFERENCE/LOW_GEAR_RATIO;
  int timeoutMs = 10;

  // Front is the follow motor, and it is based on following the primary motor of its side.
  public CANSparkMax leftDrivePrimary = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID, MotorType.kBrushless),
                     leftDriveFront = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID, MotorType.kBrushless),
                     rightDriveFront = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID, MotorType.kBrushless),
                     rightDrivePrimary = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID, MotorType.kBrushless);

  
  public CANPIDController leftPID = new CANPIDController(leftDrivePrimary);
  public CANPIDController rightPID = new CANPIDController(rightDrivePrimary);

  public CANEncoder leftEncoder = new CANEncoder(leftDrivePrimary);
  public CANEncoder rightEncoder = new CANEncoder(rightDrivePrimary);


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveCommand());

    leftDriveFront.follow(leftDrivePrimary);
    rightDriveFront.follow(rightDrivePrimary);
  
    leftDrivePrimary.setCANTimeout(timeoutMs);
    rightDrivePrimary.setCANTimeout(timeoutMs);
    leftDrivePrimary.setMotorType(MotorType.kBrushless);
    rightDrivePrimary.setMotorType(MotorType.kBrushless);

  }

  public void set(double left, double right) {
    leftDrivePrimary.set(left);
    rightDrivePrimary.set(right);
  }

  public double getLeftRevs() {
    return leftEncoder.getPosition();
  }

  public double getRightRevs() {
    return rightEncoder.getPosition();
  }

  public double lowinchesToRevs(double INCHES) {
    return INCHES/LOW_INCHES_PER_REV;
  }
  public double highinchesToRevs(double INCHES) {
    return INCHES/HIGH_INCHES_PER_REV;
  }

  // Takes an input of ticks
  public void setPID(double left, double right) {
    leftPID.setReference(left, ControlType.kPosition);
    rightPID.setReference(right, ControlType.kPosition);
  }

  public void stop() {
    leftDrivePrimary.set(0);
    rightDrivePrimary.set(0);
  }
}

package org.usfirst.frc4388.utility;

import java.util.ArrayList;

import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class MMTalonPIDController
{	
	protected static enum MMControlMode { STRAIGHT, TURN };
	public static enum MMTalonTurnType { TANK, LEFT_SIDE_ONLY, RIGHT_SIDE_ONLY };

	protected ArrayList<CANTalonEncoder> motorControllers;	
	protected long periodMs;
	protected PIDParams pidParams;	
	protected boolean useGyroLock;
	protected double startGyroAngle;
	protected double targetGyroAngle;
	protected double trackDistance;
	protected MMControlMode controlMode;
	protected MMTalonTurnType turnType;
	protected double targetValue;
	
	public MMTalonPIDController(long periodMs, PIDParams pidParams, ArrayList<CANTalonEncoder> motorControllers) 
	{
		this.motorControllers = motorControllers;
		this.periodMs = periodMs;
		setPID(pidParams);
	}
	
	private int convertInchesPerSecToTicksPer100ms(double inchesPerSec) {
		return (int)Math.round(Constants.kDriveEncoderTicksPerInch * inchesPerSec / 10);
	}
    
	public void setPID(PIDParams pidParams) {
		this.pidParams = pidParams;
		
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPID(pidParams.kP, pidParams.kI, pidParams.kD);
			//motorController.setF(pidParams.kF);
			motorController.config_kP(0, pidParams.kP, 0);	//TODO: verify want parameter slot 0, with no timeout
			motorController.config_kI(0, pidParams.kI, 0);	//TODO: verify want parameter slot 0, with no timeout
			motorController.config_kD(0, pidParams.kD, 0);	//TODO: verify want parameter slot 0, with no timeout
			motorController.config_kF(0, pidParams.kF, 0);	//TODO: verify want parameter slot 0, with no timeout
		}
	}
	
	public void setMMStraightTarget(double startValue, double targetValue, double maxVelocityInchesPerSec, double maxAccelerationInchesPerSecPerSec, boolean useGyroLock, double desiredAngle, boolean resetEncoder) {
		controlMode = MMControlMode.STRAIGHT;
		this.startGyroAngle = desiredAngle;
		this.useGyroLock = useGyroLock;
		this.targetValue = targetValue;
		
		// Set up the motion profile 
		for (CANTalonEncoder motorController : motorControllers) {
			if (resetEncoder) {
				//motorController.setPosition(0);
				motorController.setSelectedSensorPosition(0, 0, 0);	//TODO: verify want 0="Primary closed-loop", with no timeout
			}
			//motorController.setMotionMagicCruiseVelocity(maxVelocity);
			//motorController.setMotionMagicAcceleration(maxAcceleration);
			//motorController.set(targetValue);
			//motorController.changeControlMode(TalonControlMode.MotionMagic);
			motorController.configMotionCruiseVelocity(convertInchesPerSecToTicksPer100ms(maxVelocityInchesPerSec), 0);
			motorController.configMotionAcceleration(convertInchesPerSecToTicksPer100ms(maxAccelerationInchesPerSecPerSec), 0);
			motorController.set(ControlMode.MotionMagic, targetValue);
		}
	}
			
	public void setZeroPosition() {
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPosition(0);
			//motorController.set(0);
			//motorController.changeControlMode(TalonControlMode.Position);
			motorController.setSelectedSensorPosition(0, 0, 0);	//TODO: verify want 0="Primary closed-loop", with no timeout
			motorController.set(ControlMode.Position, 0);
		}
	}

	public void resetZeroPosition() {
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPosition(0);
			motorController.setSelectedSensorPosition(0, 0, 0);	//TODO: verify want 0="Primary closed-loop", with no timeout
		}
	}

	public void resetZeroPosition(double angle) {
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPosition(angle);
			motorController.setSelectedSensorPosition((int)angle, 0, 0);	//TODO URGENT: convert angle to raw sensor position
		}
	}

	private double calcTrackDistance(double deltaAngleDeg, MMTalonTurnType turnType, double trackWidth) {
		double trackDistance = deltaAngleDeg / 360.0 * Math.PI * trackWidth;
		if (turnType == MMTalonTurnType.TANK) {
			return trackDistance;
		}
		else if (turnType == MMTalonTurnType.LEFT_SIDE_ONLY) {
			return trackDistance * 2.0;
		}
		else if (turnType == MMTalonTurnType.RIGHT_SIDE_ONLY) {
			return -trackDistance * 2.0;
		}
		return 0.0;
	}
	
	public boolean controlLoopUpdate(double currentGyroAngle) {		
		// Calculate the motion profile feed forward and gyro feedback terms
		double rightTarget = 0.0;
		double leftTarget = 0.0;

		// Update the set points 
		if (controlMode == MMControlMode.STRAIGHT) {
			double gyroDelta = useGyroLock ? startGyroAngle - currentGyroAngle: 0;
			double deltaDistance = calcTrackDistance(gyroDelta, MMTalonTurnType.TANK, Drive.TRACK_WIDTH_INCHES);
			rightTarget = targetValue + deltaDistance;
			leftTarget = targetValue - deltaDistance;
			
			// Update the controllers with updated set points.
			for (CANTalonEncoder motorController : motorControllers) {
				if (motorController.isRight()) {
					//motorController.set(rightTarget);
			    	motorController.set(motorController.getControlMode(), rightTarget);	//TODO: change to explicit mode set?
				}
				else {
					//motorController.set(leftTarget);
			    	motorController.set(motorController.getControlMode(), leftTarget);	//TODO: change to explicit mode set?
				}
			}
		}
		
		return false;
	}
}
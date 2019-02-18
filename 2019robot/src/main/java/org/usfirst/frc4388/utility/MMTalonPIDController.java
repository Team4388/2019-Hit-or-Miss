package org.usfirst.frc4388.utility;

import java.util.ArrayList;

import org.usfirst.frc4388.robot.subsystems.Drive;

//import com.ctre.CANTalon.TalonControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class MMTalonPIDController
{	
	protected static enum MMControlMode { STRAIGHT, TURN };
	public static enum MMTalonTurnType { TANK, LEFT_SIDE_ONLY, RIGHT_SIDE_ONLY };

	protected ArrayList<TalonSRXEncoder> motorControllers;	
	protected long periodMs;
	protected PIDParams pidParams;	
	protected boolean useGyroLock;
	protected double startGyroAngle;
	protected double targetGyroAngle;
	protected double trackDistance;
	protected MMControlMode controlMode;
	protected MMTalonTurnType turnType;
	protected double targetValue;
	
	public MMTalonPIDController(long periodMs, PIDParams pidParams, ArrayList<TalonSRXEncoder> motorControllers) 
	{
		this.motorControllers = motorControllers;
		this.periodMs = periodMs;
		setPID(pidParams);
	}
    
	public void setPID(PIDParams pidParams) {
		this.pidParams = pidParams;
		
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPIDF(0, pidParams.kP, pidParams.kI, pidParams.kD, pidParams.kF);
		}
	}
	
	public void setMMStraightTarget(double startValue, double targetValue, double maxVelocity, double maxAcceleration, boolean useGyroLock, double desiredAngle, boolean resetEncoder) {
		controlMode = MMControlMode.STRAIGHT;
		this.startGyroAngle = desiredAngle;
		this.useGyroLock = useGyroLock;
		this.targetValue = targetValue;
		
		// Set up the motion profile 
		for (TalonSRXEncoder motorController : motorControllers) {
			if (resetEncoder) {
				motorController.setPosition(0);
			}
			motorController.configMotionCruiseVelocity((int)maxVelocity, TalonSRXEncoder.TIMEOUT_MS);
			motorController.configMotionAcceleration((int)maxAcceleration, TalonSRXEncoder.TIMEOUT_MS);
			motorController.set(ControlMode.MotionMagic, targetValue);
		}
	}
			
	public void setZeroPosition() {
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPosition(0);
			motorController.set(ControlMode.MotionMagic, targetValue);
		}
	}

	public void resetZeroPosition() {
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPosition(0);
		}
	}

	public void resetZeroPosition(double angle) {
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPositionWorld(angle);
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
			for (TalonSRXEncoder motorController : motorControllers) {
				if (motorController.isRight()) {
					motorController.set(ControlMode.MotionMagic, rightTarget);
				}
				else {
					motorController.set(ControlMode.MotionMagic, leftTarget);
				}
			}
		}
		
		return false;
	}
}
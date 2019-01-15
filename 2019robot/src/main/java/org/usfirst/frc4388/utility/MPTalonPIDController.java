
package org.usfirst.frc4388.utility;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class MPTalonPIDController
{	
	protected static enum MPControlMode { STRAIGHT, TURN };
	public static enum MPTalonTurnType { TANK, LEFT_SIDE_ONLY, RIGHT_SIDE_ONLY };

	protected ArrayList<CANTalonEncoder> motorControllers;	
	protected long periodMs;
	protected PIDParams pidParams;	
	protected MotionProfileBoxCar mp;
	protected MotionProfilePoint mpPoint;
	protected boolean useGyroLock;
	protected double startGyroAngle;
	protected double targetGyroAngle;
	protected double trackDistance;
	protected MPControlMode controlMode;
	protected MPTalonTurnType turnType;
	
	public MPTalonPIDController(long periodMs, PIDParams pidParams, ArrayList<CANTalonEncoder> motorControllers) 
	{
		this.motorControllers = motorControllers;
		this.periodMs = periodMs;
		setPID(pidParams);
	}
    
	public void setPID(PIDParams pidParams) {
		this.pidParams = pidParams;
		
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPID(pidParams.kP, pidParams.kI, pidParams.kD);
			motorController.config_kP(0, pidParams.kP, 0);	//TODO: verify want parameter slot 0, with no timeout
			motorController.config_kI(0, pidParams.kI, 0);	//TODO: verify want parameter slot 0, with no timeout
			motorController.config_kD(0, pidParams.kD, 0);	//TODO: verify want parameter slot 0, with no timeout
		}
	}
	
	public void setMPTarget(double startValue, double targetValue, double maxVelocity, double t1, double t2) {
		setMPStraightTarget(startValue, targetValue, maxVelocity, t1, t2, false, 0, false);
	}

	public void setMPTarget(double startValue, double targetValue, double maxVelocity, double t1, double t2, boolean resetEncoder) {
		setMPStraightTarget(startValue, targetValue, maxVelocity, t1, t2, false, 0, resetEncoder);
	}

	public void setMPStraightTarget(double startValue, double targetValue, double maxVelocity, double t1, double t2, boolean useGyroLock, double desiredAngle, boolean resetEncoder) {
		controlMode = MPControlMode.STRAIGHT;
		this.startGyroAngle = desiredAngle;
		this.useGyroLock = useGyroLock;
		
		// Set up the motion profile 
		mp = new MotionProfileBoxCar(startValue, targetValue, maxVelocity, periodMs, t1, t2);
		for (CANTalonEncoder motorController : motorControllers) {
			if (resetEncoder) {
				//motorController.setPosition(0);
				motorController.setSelectedSensorPosition(0, 0, 0);	//TODO: verify want 0="Primary closed-loop", with no timeout
			}
			//motorController.set(mp.getStartDistance());
			//motorController.changeControlMode(TalonControlMode.Position);
			motorController.set(ControlMode.Position, mp.getStartDistance());
		}
	}
	
	//public void setMPStraightTarget(String key, boolean useGyroLock, double desiredAngle, boolean resetEncoder) {
	//	controlMode = MPControlMode.STRAIGHT;
	//	this.startGyroAngle = desiredAngle;
	//	this.useGyroLock = useGyroLock;
	//	
	//	// Set up the motion profile 
	//	mp = MotionProfileCache.getInstance().getMP(key);
	//	for (CANTalonEncoder motorController : motorControllers) {
	//		if (resetEncoder) {
	//			motorController.setPosition(0);
	//		}
	//		motorController.set(mp.getStartDistance());
	//		motorController.changeControlMode(TalonControlMode.Position);
	//	}
	//}
	
	public void setMPTurnTarget(double startAngleDeg, double targetAngleDeg, double maxTurnRateDegPerSec, double t1, double t2, MPTalonTurnType turnType, double trackWidth) {
		controlMode = MPControlMode.TURN;
		this.turnType = turnType;
		this.startGyroAngle = startAngleDeg;
		this.targetGyroAngle = targetAngleDeg;
		this.useGyroLock = true;
		
		trackDistance = calcTrackDistance(targetAngleDeg - startAngleDeg, turnType, trackWidth);

		// Set up the motion profile 
		mp = new MotionProfileBoxCar(0, trackDistance, maxTurnRateDegPerSec, periodMs, t1, t2);
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPosition(0);
			//motorController.set(0);
			//motorController.changeControlMode(TalonControlMode.Position);
			motorController.setSelectedSensorPosition(0, 0, 0);	//TODO: verify want 0="Primary closed-loop", with no timeout
			motorController.set(ControlMode.Position, 0);
		}
		
		if (Math.abs(trackDistance) < 0.0001) {
			trackDistance = 1;
		}
	}
		
	private double calcTrackDistance(double deltaAngleDeg, MPTalonTurnType turnType, double trackWidth) {
		double trackDistance = deltaAngleDeg / 360.0 * Math.PI * trackWidth;
		if (turnType == MPTalonTurnType.TANK) {
			return trackDistance;
		}
		else if (turnType == MPTalonTurnType.LEFT_SIDE_ONLY) {
			return trackDistance * 2.0;
		}
		else if (turnType == MPTalonTurnType.RIGHT_SIDE_ONLY) {
			return -trackDistance * 2.0;
		}
		return 0.0;
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

	public boolean controlLoopUpdate() {
		return controlLoopUpdate(0);
	}
	
	public boolean controlLoopUpdate(double currentGyroAngle) {
		mpPoint = mp.getNextPoint(mpPoint);
		
		// Check if we are finished
		if (mpPoint == null) {
			return true;
		}
		
		// Calculate the motion profile feed forward and gyro feedback terms
		double KfLeft = 0.0;
		double KfRight = 0.0;

		// Update the set points and Kf gains
		if (controlMode == MPControlMode.STRAIGHT) {
			double gyroDelta = useGyroLock ? startGyroAngle - currentGyroAngle: 0;
			if (Math.abs(mpPoint.position) > 0.001) {
				//KfLeft = (pidParams.kA * mpPoint.acceleration + pidParams.kV * mpPoint.velocity + pidParams.kG * gyroDelta) / mpPoint.position;
				//KfRight = (pidParams.kA * mpPoint.acceleration + pidParams.kV * mpPoint.velocity - pidParams.kG * gyroDelta) / mpPoint.position;
			
				KfLeft = -(pidParams.kA * mpPoint.acceleration + pidParams.kV * mpPoint.velocity + pidParams.kG * gyroDelta) / mpPoint.position;
				KfRight = (pidParams.kA * mpPoint.acceleration + pidParams.kV * mpPoint.velocity - pidParams.kG * gyroDelta) / mpPoint.position;
			}
			
			// Update the controllers Kf and set point.
			for (CANTalonEncoder motorController : motorControllers) {
				if (motorController.isRight()) {
					//motorController.setF(KfRight);
					motorController.config_kF(0, KfRight, 0);	//TODO: verify
					motorController.setWorld(-mpPoint.position);
				}
				else {
					//motorController.setF(KfLeft);
					motorController.config_kF(0, KfLeft, 0);	//TODO: verify
					motorController.setWorld(mpPoint.position);
				}
			}
		}
		
		else {
			double mpAngle = startGyroAngle + ((targetGyroAngle - startGyroAngle) * mpPoint.position / trackDistance);
			double gyroDelta = mpAngle - currentGyroAngle;
			if (Math.abs(mpPoint.position) > 0.001) {
				KfLeft = (pidParams.kA * mpPoint.acceleration + pidParams.kV * mpPoint.velocity + pidParams.kG * gyroDelta) / mpPoint.position;
				KfRight = (pidParams.kA * mpPoint.acceleration + pidParams.kV * mpPoint.velocity + pidParams.kG * gyroDelta) / mpPoint.position;
			}
			
			for (CANTalonEncoder motorController : motorControllers) {
				if (turnType == MPTalonTurnType.TANK) {
					if (motorController.isRight()) {
						//motorController.setF(KfRight);
						motorController.config_kF(0, KfRight, 0);	//TODO: verify
						motorController.setWorld(-mpPoint.position);
					}
					else {
						//motorController.setF(KfLeft);
						motorController.config_kF(0, KfLeft, 0);	//TODO: verify
						motorController.setWorld(mpPoint.position);
					}
				}
				else if (turnType == MPTalonTurnType.LEFT_SIDE_ONLY) {
					if (!motorController.isRight()) {
						//motorController.setF(KfLeft);
						motorController.config_kF(0, KfLeft, 0);	//TODO: verify
						motorController.setWorld(mpPoint.position);
					}
				}
				else if (turnType == MPTalonTurnType.RIGHT_SIDE_ONLY) {
					if (motorController.isRight()) {
						//motorController.setF(KfRight);
						motorController.config_kF(0, KfRight, 0);	//TODO: verify
						motorController.setWorld(-mpPoint.position);
					}
				}
			}
		}
		
		return false;
	}
	
	public MotionProfilePoint getCurrentPoint() {
		return mpPoint;
	}
}

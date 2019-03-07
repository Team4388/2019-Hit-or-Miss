package org.usfirst.frc4388.utility;

import java.util.ArrayList;

//import com.ctre.CANTalon.TalonControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class MPTalonPIDController
{	
	protected static enum MPControlMode { STRAIGHT, TURN };
	public static enum MPTalonTurnType { TANK, LEFT_SIDE_ONLY, RIGHT_SIDE_ONLY };
	protected ArrayList<TalonSRXEncoder> motorControllers;	
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
	protected int pidSlot;
	
	public MPTalonPIDController(long periodMs, ArrayList<TalonSRXEncoder> motorControllers) 
	{
		this.motorControllers = motorControllers;
		this.periodMs = periodMs;
	}
    
	public void setPID(PIDParams pidParams, int slot) {
		this.pidParams = pidParams;
		
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPID(slot, pidParams.kP, pidParams.kI, pidParams.kD);
		}
	}
	
	public void setPIDSlot(int slot) {
		this.pidSlot = slot;
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.selectProfileSlot(slot, 0);
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
		for (TalonSRXEncoder motorController : motorControllers) {
			if (resetEncoder) {
				motorController.setPosition(0);
			}
			motorController.setWorld(ControlMode.Position, mp.getStartDistance());
		}
	}
	
	public double getStartPosition() {
		return mp != null ? mp.getStartDistance() : 0;
	}
	
	public double getTargetPosition() {
		return mp != null ? mp.getTargetDistance() : 0;
	}
	
	public void setMPStraightTarget(String key, boolean useGyroLock, double desiredAngle, boolean resetEncoder) {
		controlMode = MPControlMode.STRAIGHT;
		this.startGyroAngle = desiredAngle;
		this.useGyroLock = useGyroLock;
		
		// Set up the motion profile 
		mp = MotionProfileCache.getInstance().getMP(key);
		for (TalonSRXEncoder motorController : motorControllers) {
			if (resetEncoder) {
				motorController.setPosition(0);
			}
			motorController.setWorld(ControlMode.Position, mp.getStartDistance());
		}
	}
	
	public void setMPTurnTarget(double startAngleDeg, double targetAngleDeg, double maxTurnRateDegPerSec, double t1, double t2, MPTalonTurnType turnType, double trackWidth) {
		controlMode = MPControlMode.TURN;
		this.turnType = turnType;
		this.startGyroAngle = startAngleDeg;
		this.targetGyroAngle = targetAngleDeg;
		this.useGyroLock = true;
		
		trackDistance = calcTrackDistance(targetAngleDeg - startAngleDeg, turnType, trackWidth);

		// Set up the motion profile 
		mp = new MotionProfileBoxCar(0, trackDistance, maxTurnRateDegPerSec, periodMs, t1, t2);
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPosition(0);
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
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPosition(0);
			motorController.set(ControlMode.Position, 0);
		}
	}

	public void resetZeroPosition() {
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPosition(0);
		}
	}

	public void resetZeroPosition(double position) {
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPositionWorld(position);
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
				KfLeft = (pidParams.kA * mpPoint.acceleration + pidParams.kV * mpPoint.velocity + pidParams.kG * gyroDelta) / mpPoint.position;
				KfRight = (pidParams.kA * mpPoint.acceleration + pidParams.kV * mpPoint.velocity - pidParams.kG * gyroDelta) / mpPoint.position;
			}
			
			// Update the controllers Kf and set point.
			for (TalonSRXEncoder motorController : motorControllers) {
				if (motorController.isRight()) {
					motorController.config_kF(0, KfRight, TalonSRXEncoder.TIMEOUT_MS);
					motorController.setWorld(ControlMode.Position, mpPoint.position);
				}
				else {
					motorController.config_kF(0, KfLeft, TalonSRXEncoder.TIMEOUT_MS);
					motorController.setWorld(ControlMode.Position, mpPoint.position);
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
			
			for (TalonSRXEncoder motorController : motorControllers) {
				if (turnType == MPTalonTurnType.TANK) {
					if (motorController.isRight()) {
						motorController.config_kF(0, KfRight, TalonSRXEncoder.TIMEOUT_MS);
						motorController.setWorld(ControlMode.Position, -mpPoint.position);
					}
					else {
						motorController.config_kF(0, KfLeft, TalonSRXEncoder.TIMEOUT_MS);
						motorController.setWorld(ControlMode.Position, mpPoint.position);
					}
				}
				else if (turnType == MPTalonTurnType.LEFT_SIDE_ONLY) {
					if (!motorController.isRight()) {
						motorController.config_kF(0, KfLeft, TalonSRXEncoder.TIMEOUT_MS);
						motorController.setWorld(ControlMode.Position, mpPoint.position);
					}
				}
				else if (turnType == MPTalonTurnType.RIGHT_SIDE_ONLY) {
					if (motorController.isRight()) {
						motorController.config_kF(0, KfRight, TalonSRXEncoder.TIMEOUT_MS);
						motorController.setWorld(ControlMode.Position, -mpPoint.position);
					}
				}
			}
		}
		
		return false;
	}
	
	public void setTarget(double position, double Kf) {
		// Kf gets multipled by position in the Talon 
		double KfPerPosition = Math.abs(position) > 0.001 ? Kf / position : 0;

		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.config_kF(0, KfPerPosition, TalonSRXEncoder.TIMEOUT_MS);
			motorController.setWorld(ControlMode.Position, position);
			//System.err.println(motorController.getControlMode());
		}
	}
	
	public MotionProfilePoint getCurrentPoint() {
		return mpPoint;
	}
}
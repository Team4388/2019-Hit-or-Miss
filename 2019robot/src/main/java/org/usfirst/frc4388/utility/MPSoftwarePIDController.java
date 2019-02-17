package org.usfirst.frc4388.utility;

import java.util.ArrayList;

//import com.ctre.CANTalon.TalonControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class MPSoftwarePIDController
{	
	public static enum MPSoftwareTurnType { TANK, LEFT_SIDE_ONLY, RIGHT_SIDE_ONLY, LEFT_ARC, RIGHT_ARC };

	protected ArrayList<TalonSRXEncoder> motorControllers;	
	protected long periodMs;
	protected PIDParams pidParams;	
	protected MotionProfileBoxCar mp;
	protected MotionProfilePoint mpPoint;
	protected boolean useGyroLock;
	protected double startGyroAngle;
	protected double targetGyroAngle;
	protected MPSoftwareTurnType turnType;
	
	protected double prevError = 0.0; // the prior error (used to compute velocity)
	protected double totalError = 0.0; // the sum of the errors for use in the integral calc
	
	public MPSoftwarePIDController(long periodMs, PIDParams pidParams, ArrayList<TalonSRXEncoder> motorControllers) 
	{
		this.motorControllers = motorControllers;
		this.periodMs = periodMs;
		setPID(pidParams);
	}
    
	public void setPID(PIDParams pidParams) {
		this.pidParams = pidParams;
	}
	
	public void setMPTurnTarget(double startAngleDeg, double targetAngleDeg, double maxTurnRateDegPerSec, double t1, double t2, MPSoftwareTurnType turnType, double trackWidth) {
		this.turnType = turnType;
		this.startGyroAngle = startAngleDeg;
		this.targetGyroAngle = targetAngleDeg;
		this.useGyroLock = true;
		
		// Set up the motion profile 
		mp = new MotionProfileBoxCar(startAngleDeg, targetAngleDeg, maxTurnRateDegPerSec, periodMs, t1, t2);		
		prevError = 0.0;
		totalError = 0.0;
	}
	
	public void setMPTurnTarget(String key, MPSoftwareTurnType turnType, double trackWidth) {
		this.turnType = turnType;
		this.useGyroLock = true;
		
		// Set up the motion profile 
		mp = MotionProfileCache.getInstance().getMP(key);
		this.startGyroAngle = mp.getStartDistance();
		this.targetGyroAngle = mp.getTargetDistance();
				
		prevError = 0.0;
		totalError = 0.0;
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
		
		// Calculate the motion profile feed forward and error feedback terms
		double error = mpPoint.position - currentGyroAngle;
		
		if (Math.abs(targetGyroAngle - currentGyroAngle) < pidParams.iZone) {
			totalError += error;
		}
		else {
			totalError = 0;
		}
		
		double output =  pidParams.kP * error + pidParams.kI * totalError + pidParams.kD * (error - prevError) + pidParams.kA * mpPoint.acceleration + pidParams.kV * mpPoint.velocity;
		prevError = error;
			
		// Update the controllers set point.
		if (turnType == MPSoftwareTurnType.TANK) {
			for (TalonSRXEncoder motorController : motorControllers) {
				if (motorController.isRight()) {
					motorController.set(ControlMode.PercentOutput, -output);
				}
				else {
					motorController.set(ControlMode.PercentOutput, output);					
				}
			}
		}
		else if (turnType == MPSoftwareTurnType.LEFT_SIDE_ONLY) {
			for (TalonSRXEncoder motorController : motorControllers) {
				if (motorController.isRight()) {
					motorController.set(ControlMode.PercentOutput, 0);
				}
				else {
					motorController.set(ControlMode.PercentOutput, 2.0 * output);					
				}
			}
		}
		else if (turnType == MPSoftwareTurnType.RIGHT_SIDE_ONLY) {
			for (TalonSRXEncoder motorController : motorControllers) {
				if (motorController.isRight()) {
					motorController.set(ControlMode.PercentOutput, 2.0 * output);
				}
				else {
					motorController.set(ControlMode.PercentOutput, 0);					
				}
			}
		}
		else if (turnType == MPSoftwareTurnType.LEFT_ARC) {
			for (TalonSRXEncoder motorController : motorControllers) {
				if (motorController.isRight()) {
					motorController.set(ControlMode.PercentOutput, 1.0 * output);
				}
				else {
					motorController.set(ControlMode.PercentOutput, 2.0 * output);					
				}
			}
		}
		else if (turnType == MPSoftwareTurnType.RIGHT_ARC) {
			for (TalonSRXEncoder motorController : motorControllers) {
				if (motorController.isRight()) {
					motorController.set(ControlMode.PercentOutput, 2.0 * output);
				}
				else {
					motorController.set(ControlMode.PercentOutput, 1.0 * output);					
				}
			}
		}

		return false;
	}
	
	public MotionProfilePoint getCurrentPoint() {
		return mpPoint;
	}
}
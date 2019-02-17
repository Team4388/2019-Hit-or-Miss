package org.usfirst.frc4388.utility;

import java.util.ArrayList;

//import com.ctre.CANTalon.TalonControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class MPTalonPIDPathController
{	
	protected ArrayList<TalonSRXEncoder> motorControllers;	
	protected long periodMs;
	protected PIDParams pidParams;	
	protected double startGyroAngle;
	protected double targetGyroAngle;
	protected double trackDistance;
	
	public MPTalonPIDPathController(long periodMs, PIDParams pidParams, ArrayList<TalonSRXEncoder> motorControllers) 
	{
		this.motorControllers = motorControllers;
		this.periodMs = periodMs;
		setPID(pidParams);
	}
    
	public void setPID(PIDParams pidParams) {
		this.pidParams = pidParams;
		
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPID(0, pidParams.kP, pidParams.kI, pidParams.kD);
		}
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

	public void resetZeroPosition(double angle) {
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPositionWorld(angle);
		}
	}

	public boolean controlLoopUpdate(double currentGyroAngle) {
		
		// Calculate the motion profile feed forward and gyro feedback terms
		double KfLeft = 0.0;
		double KfRight = 0.0;

		// Update the set points and Kf gains
		
		// Update the controllers Kf and set point.
		for (TalonSRXEncoder motorController : motorControllers) {
			if (motorController.isRight()) {
				motorController.config_kF(0, KfRight, TalonSRXEncoder.TIMEOUT_MS);
//				motorController.setWorld(ControlMode.Position, rightPoint.position);
			}
			else {
				motorController.config_kF(0, KfLeft, TalonSRXEncoder.TIMEOUT_MS);
			}
		}
		
		
		return false;
	}
}
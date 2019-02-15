package org.usfirst.frc4388.utility;

import java.util.ArrayList;

//import com.ctre.CANTalon.TalonControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class MPTalonPIDPathVelocityController
{	
	protected ArrayList<TalonSRXEncoder> motorControllers;	
	protected long periodMs;
	protected PIDParams pidParams;	
//	protected PathGenerator path;
	protected double startGyroAngle;
	protected double targetGyroAngle;
	protected double trackDistance;
	
	public MPTalonPIDPathVelocityController(long periodMs, PIDParams pidParams, ArrayList<TalonSRXEncoder> motorControllers) 
	{
		this.motorControllers = motorControllers;
		this.periodMs = periodMs;
		setPID(pidParams);
	}
    
	public void setPID(PIDParams pidParams) {
		this.pidParams = pidParams;
		
		for (TalonSRXEncoder motorController : motorControllers) {
			motorController.setPIDF(0, pidParams.kP, pidParams.kI, pidParams.kD, pidParams.kF);
			motorController.configVoltageCompSaturation(12.0, TalonSRXEncoder.TIMEOUT_MS);
			motorController.enableVoltageCompensation(true);
			motorController.configNominalOutputForward(0.0, TalonSRXEncoder.TIMEOUT_MS);
			motorController.configNominalOutputReverse(0.0, TalonSRXEncoder.TIMEOUT_MS);
			motorController.configPeakOutputForward(+1.0f, TalonSRXEncoder.TIMEOUT_MS);
			motorController.configPeakOutputReverse(-1.0f, TalonSRXEncoder.TIMEOUT_MS);
			motorController.selectProfileSlot(0, TalonSRXEncoder.PID_IDX);
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
//	}

		
		// Update the controllers Kf and set point.
		for (TalonSRXEncoder motorController : motorControllers) {
			if (motorController.isRight()) {
			}
			else {
			}
		}
		
	}
}
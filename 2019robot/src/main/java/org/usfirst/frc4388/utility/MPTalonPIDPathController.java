package org.usfirst.frc4388.utility;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import jaci.pathfinder.Trajectory.Segment;

public class MPTalonPIDPathController
{	
	protected ArrayList<CANTalonEncoder> motorControllers;	
	protected long periodMs;
	protected PIDParams pidParams;	
	protected PathGenerator path;
	protected double startGyroAngle;
	protected double targetGyroAngle;
	protected double trackDistance;
	
	public MPTalonPIDPathController(long periodMs, PIDParams pidParams, ArrayList<CANTalonEncoder> motorControllers) 
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
	
	public void setMPPathTarget(PathGenerator path) {
		this.path = path;
		path.resetCounter();
		
		// Set up the motion profile 
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPosition(0);
			//motorController.set(0);
			//motorController.changeControlMode(TalonControlMode.Position);
			motorController.setSelectedSensorPosition(0, 0, 0);	//TODO: verify want 0="Primary closed-loop", with no timeout
			motorController.set(ControlMode.Position, 0);
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

	public boolean controlLoopUpdate(double currentGyroAngle) {
		Segment leftPoint = path.getLeftPoint();
		Segment rightPoint = path.getRightPoint();
		
		// Check if we are finished
		if (leftPoint == null) {
			return true;
		}
		
		// Calculate the motion profile feed forward and gyro feedback terms
		double KfLeft = 0.0;
		double KfRight = 0.0;

		// Update the set points and Kf gains
		double gyroDelta = -path.getHeading() - currentGyroAngle;
		if (Math.abs(leftPoint.position) > 0.001) {
			KfLeft = (pidParams.kA * leftPoint.acceleration + pidParams.kV * leftPoint.velocity + pidParams.kG * gyroDelta) / leftPoint.position;
			KfRight = (pidParams.kA * rightPoint.acceleration + pidParams.kV * rightPoint.velocity - pidParams.kG * gyroDelta) / rightPoint.position;
		}
		
		// Update the controllers Kf and set point.
		for (CANTalonEncoder motorController : motorControllers) {
			if (motorController.isRight()) {
				//motorController.setF(KfRight);
				motorController.config_kF(0, KfRight, 0);	//TODO: verify
				motorController.setWorld(rightPoint.position);
			}
			else {
				//motorController.setF(KfLeft);
				motorController.config_kF(0, KfLeft, 0);	//TODO: verify
				motorController.setWorld(leftPoint.position);
			}
		}
		
		path.incrementCounter();
		
		return false;
	}
}
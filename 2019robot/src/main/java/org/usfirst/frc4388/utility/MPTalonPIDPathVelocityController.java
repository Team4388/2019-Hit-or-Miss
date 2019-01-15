package org.usfirst.frc4388.utility;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import jaci.pathfinder.Trajectory.Segment;

public class MPTalonPIDPathVelocityController
{	
	protected ArrayList<CANTalonEncoder> motorControllers;	
	protected long periodMs;
	protected PIDParams pidParams;	
	protected PathGenerator path;
	protected double startGyroAngle;
	protected double targetGyroAngle;
	protected double trackDistance;
	
	public MPTalonPIDPathVelocityController(long periodMs, PIDParams pidParams, ArrayList<CANTalonEncoder> motorControllers) 
	{
		this.motorControllers = motorControllers;
		this.periodMs = periodMs;
		setPID(pidParams);
	}
    
	public void setPID(PIDParams pidParams) {
		this.pidParams = pidParams;
		
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPID(pidParams.kP, pidParams.kI, pidParams.kD);
			//motorController.setF(pidParams.kF);
			//motorController.configNominalOutputVoltage(+0.0f, -0.0f);
			//motorController.configPeakOutputVoltage(+12.0f, -12.0f);
			//motorController.setProfile(0);
			motorController.config_kP(0, pidParams.kP, 0);
			motorController.config_kI(0, pidParams.kI, 0);
			motorController.config_kD(0, pidParams.kD, 0);
			motorController.config_kF(0, pidParams.kF, 0);
			motorController.configNominalOutputForward(+0.0f, 0);
			motorController.configNominalOutputReverse(-0.0f, 0);
			motorController.configPeakOutputForward(+1.0f, 0);
			motorController.configPeakOutputReverse(-1.0f, 0);
			motorController.selectProfileSlot(0, 0);
		}
	}
	
	public void setMPPathTarget(PathGenerator path) {
		this.path = path;
		path.resetCounter();
		
		// Set up the motion profile 
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPosition(0);
			//motorController.set(0);
			//motorController.changeControlMode(TalonControlMode.Speed);
			motorController.setSelectedSensorPosition(0, 0, 0);	//TODO: verify want 0="Primary closed-loop", with no timeout
			motorController.set(ControlMode.Velocity, 0);
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
		double rightVelocity = rightPoint.velocity;
		double leftVelocity = leftPoint.velocity;
		
		// Update the controllers Kf and set point.
		for (CANTalonEncoder motorController : motorControllers) {
			if (motorController.isRight()) {
				motorController.setVelocityWorld(rightVelocity);
			}
			else {
				motorController.setVelocityWorld(leftVelocity);
			}
		}
		
		path.incrementCounter();
		
		return false;
	}
}
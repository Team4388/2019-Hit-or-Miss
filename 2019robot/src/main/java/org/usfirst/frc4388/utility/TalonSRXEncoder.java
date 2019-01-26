package org.usfirst.frc4388.utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonSRXEncoder extends WPI_TalonSRX
{
	public static int TIMEOUT_MS = 0;
	public static int PID_IDX = 0;

	private double encoderTicksToWorld;
	private boolean isRight = true;
	
	public TalonSRXEncoder(int deviceId, double encoderTicksToWorld, FeedbackDevice feedbackDevice) {
		this(deviceId, encoderTicksToWorld, false, feedbackDevice);
	}

	public TalonSRXEncoder(int deviceNumber, double encoderTicksToWorld, boolean isRight, FeedbackDevice feedbackDevice) {
		super(deviceNumber);
		this.configSelectedFeedbackSensor(feedbackDevice, PID_IDX, TIMEOUT_MS);
		this.encoderTicksToWorld = encoderTicksToWorld;
		this.isRight = isRight;
	}

    public boolean isRight() {
		return isRight;
	}

	public void setRight(boolean isRight) {
		this.isRight = isRight;
	}

	public void setPID(int slotId, double kP, double kI, double kD) {
		this.setPIDF(slotId, kP, kI, kD, 0);
	}

	public void setPIDF(int slotId, double kP, double kI, double kD, double kF) {
		this.config_kP(slotId, kP, TIMEOUT_MS);
		this.config_kI(slotId, kI, TIMEOUT_MS);
		this.config_kD(slotId, kD, TIMEOUT_MS);
		this.config_kF(slotId, kF, TIMEOUT_MS);
	}
	
	public void setPIDFIZone(int slotId, double kP, double kI, double kD, double kF, int iZone) {
		this.config_kP(slotId, kP, TIMEOUT_MS);
		this.config_kI(slotId, kI, TIMEOUT_MS);
		this.config_kD(slotId, kD, TIMEOUT_MS);
		this.config_kF(slotId, kF, TIMEOUT_MS);
		this.config_IntegralZone(slotId, iZone, TIMEOUT_MS);
	}
	
	public double convertEncoderTicksToWorld(double encoderTicks) {
    	return encoderTicks / encoderTicksToWorld;
    }

    public int convertEncoderWorldToTicks(double worldValue) {
    	return (int)(worldValue * encoderTicksToWorld);
    }
    
    public void setWorld(ControlMode mode, double worldValue) {
    	this.set(mode, convertEncoderWorldToTicks(worldValue));
    }
    
    public void setPosition(int value) {
    	this.setSelectedSensorPosition(value, PID_IDX, TIMEOUT_MS);
    }
    
    public void setPositionWorld(double worldValue) {
    	this.setSelectedSensorPosition(convertEncoderWorldToTicks(worldValue), PID_IDX, TIMEOUT_MS);
    }
    
    public double getPositionWorld() {
    	return convertEncoderTicksToWorld(this.getSelectedSensorPosition(PID_IDX));
    }
    
    public void setVelocityWorld(double worldValue) {
    	this.set(ControlMode.Velocity, convertEncoderWorldToTicks(worldValue) * 0.1);
    }
    
    public double getVelocityWorld() {
    	return convertEncoderTicksToWorld(this.getSelectedSensorVelocity(PID_IDX) / 0.1);
    }
}
package org.usfirst.frc4388.utility;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class CANTalonEncoder extends WPI_TalonSRX
{
	private double encoderTicksToWorld;
	private boolean isRight = true;
	

	
	public CANTalonEncoder(int deviceNumber, double encoderTicksToWorld, FeedbackDevice feedbackDevice) {
		this(deviceNumber, encoderTicksToWorld, false, feedbackDevice);
	}

	public CANTalonEncoder(int deviceNumber, double encoderTicksToWorld, boolean isRight, FeedbackDevice feedbackDevice) {
		super(deviceNumber);
		//this.setFeedbackDevice(feedbackDevice);
		this.configSelectedFeedbackSensor(feedbackDevice, 0, 0);
		this.encoderTicksToWorld = encoderTicksToWorld;
		this.isRight = isRight;
	}

    public boolean isRight() {
		return isRight;
	}

	public void setRight(boolean isRight) {
		this.isRight = isRight;
	}

	public double convertEncoderTicksToWorld(double encoderTicks) {
    	return encoderTicks / encoderTicksToWorld;
    }

    public double convertEncoderWorldToTicks(double worldValue) {
    	return worldValue * encoderTicksToWorld;
    }
    
    public void setWorld(double worldValue) {
    	//this.set(convertEncoderWorldToTicks(worldValue));
    	this.set(getControlMode(), convertEncoderWorldToTicks(worldValue));		//TODO: change to explicit mode set?
    }
    
    public void setPositionWorld(double worldValue) {
    	//this.setPosition(convertEncoderWorldToTicks(worldValue));
    	this.setSelectedSensorPosition((int)convertEncoderWorldToTicks(worldValue), 0, 0);	//TODO: verify
    }
    
    public double getPositionWorld() {
    	//return convertEncoderTicksToWorld(this.getPosition());
    	return convertEncoderTicksToWorld(this.getSelectedSensorPosition(0));	//TODO: verify want 0="Primary closed-loop"
    }
    
    public void setVelocityWorld(double worldValue) {
    	//this.set(convertEncoderWorldToTicks(worldValue) * 0.1);
    	this.set(getControlMode(), convertEncoderWorldToTicks(worldValue) * 0.1);	//TODO: change to explicit mode set?
    }
    
    public double getVelocityWorld() {
    	//return convertEncoderTicksToWorld(this.getSpeed() / 0.1);
    	return convertEncoderTicksToWorld(this.getSelectedSensorVelocity(0) / 0.1);	//TODO: verify want 0="Primary closed-loop"
    }
}

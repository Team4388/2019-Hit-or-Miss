
package org.usfirst.frc4388.utility;

import java.util.ArrayList;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;

public class SoftwarePIDPositionController
{	
	//protected ArrayList<CANTalonEncoder> motorControllers;	
	protected WPI_TalonSRX motorController;
	protected PIDParams pidParams;
	
	protected PIDParams PValue;
	
	protected double targetEncoderPosition;
	
	protected double minTurnOutput = 0.002;
	protected double maxError;
	protected double minError;
	protected double maxPrevError; ///??
	protected double prevError = 0.0; // the prior error (used to compute velocity)
	protected double totalError = 0.0; // the sum of the errors for use in the integral calc
	
	public SoftwarePIDPositionController(PIDParams PValue, WPI_TalonSRX elevatorLeft) 
	{
		this.motorController = elevatorLeft;
		setP(PValue);
	}
    
	public void setP(PIDParams PValue) 
	{
		this.PValue = PValue;
	}
	
	public void setPIDPositionTarget(double targetPosition, double maxError, double minError)
	{
		this.targetEncoderPosition = targetPosition;
		
		this.maxError = maxError;
		this.minError = minError;
		//this.maxPrevError = maxPrevError;
		
		prevError = 0.0;
		totalError = 0.0;
	}
	
	public boolean controlLoopUpdate() 
	{
		return controlLoopUpdate(0);
	}
	
	public boolean controlLoopUpdate(double currentEncoderPosition) 
	{
		// Calculate the motion profile feed forward and error feedback terms
		double error = targetEncoderPosition - currentEncoderPosition;
		//double deltaLastError = (error - prevError);
		
		//Updating the error
		//totalError += error;
		
		// Check if we are finished
		if (Math.abs(error) < maxError && Math.abs(error) > minError) 
		{
			//Robot.elevator.holdInPos();
			
			return true;
		}
		
		double output =  pidParams.kP * error; // + pidParams.kI * totalError + pidParams.kD * deltaLastError;
		
		prevError = error;
		
		// Update the controllers set point.
		motorController.set(ControlMode.PercentOutput, output);
		
		return false;
	}
}
package org.usfirst.frc4388.robot.subsystems;

import java.util.ArrayList;

import org.usfirst.frc4388.controller.XboxController;
import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.OI;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.robot.commands.*;
import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.utility.TalonSRXEncoder;
import org.usfirst.frc4388.utility.Looper;
import org.usfirst.frc4388.utility.PIDParams;
import org.usfirst.frc4388.utility.SoftwarePIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 *
 */
public class Climber extends Subsystem{
	
	//Motors
	private WPI_TalonSRX climberBack;
	private WPI_TalonSRX climberFront;
	private WPI_TalonSRX climberFront2;
	private WPI_TalonSRX flipOutMotor;

	//Frequency Control
	static float BACK_FREQ = 1;
	static float FRONT_FREQ = 1;
	static float FREQ_RATIO = 0.2443744576F;

	//Limit and Saftey vars
	LimitSwitchSource limitSwitchSource;
	SensorCollection isPressed;
	boolean safetySwitch;

	//Climb Trigger
	double rightTriggerInput;
	double leftTriggerInput;
	
	public Climber(){
		
		try{
			climberBack = new WPI_TalonSRX(RobotMap.CLIMBER_CAN_ID);
			climberFront = new WPI_TalonSRX(RobotMap.CLIMBER_WHEEL1_ID);
			climberFront2 = new WPI_TalonSRX(RobotMap.CLIMBER_WHEEL2_ID);
			climberFront2.set(ControlMode.Follower, climberFront.getDeviceID());

			climberBack.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			climberBack.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			climberBack.setNeutralMode(NeutralMode.Brake);
			climberFront.setNeutralMode(NeutralMode.Brake);
			FRONT_FREQ = BACK_FREQ * FREQ_RATIO; // Sets the front motor speed to ~1/4 the back motor speed
		} 
		catch (Exception e) {
			System.err.println("The code broke before the guard did. An error occurred in the climbing constructor");
		}
	}

    @Override
    public void initDefaultCommand() {

    }


    @Override
    public void periodic() {

		rightTriggerInput = OI.getInstance().getDriverController().getRightTriggerAxis();
		leftTriggerInput = OI.getInstance().getDriverController().getLeftTriggerAxis();
		double speed = rightTriggerInput - leftTriggerInput;

		// Put code here to be run every loop
		if (safetySwitch) {
			/*
			if (isPressed.isRevLimitSwitchClosed() && speed < 0){ //If leg at min height, and the input would retract the leg
				climberBack.set(0);
				climberFront.set(0);
			}
			else if(isPressed.isFwdLimitSwitchClosed() && speed > 0){ //If leg at max height, and the input would extend the leg
				climberBack.set(0);
				climberFront.set(FRONT_FREQ * speed);
			} */
			if (speed < 0){
				climberBack.set(-0.3 * speed);
			} 
			else { //If leg not at max height
				climberBack.set(-0.5 * speed);
				climberFront.set(speed);
			}		
		}
		else {
			climberBack.set(0);
		}
	}
	
	///DEPRICATED
	public void setClimbSpeed(double speed) {
		if (safetySwitch) {
			if (isPressed.isRevLimitSwitchClosed() && speed < 0){ //If leg at min height, and the input would retract the leg
				climberBack.set(0);
				climberFront.set(0);
			}
			else if(isPressed.isFwdLimitSwitchClosed() && speed > 0){ //If leg at max height, and the input would extend the leg
				climberBack.set(0);
				climberFront.set(FRONT_FREQ * speed);
			}
			else { //If leg not at max height */
				climberBack.set(BACK_FREQ * speed);
				climberFront.set(FRONT_FREQ * speed);
			}		
		}
	}

	public void safetySwitch(boolean safetySwitch){
		this.safetySwitch = safetySwitch;
	}

	public void flipRatchet(double speed){
		climberFront.set(speed);
	}
}

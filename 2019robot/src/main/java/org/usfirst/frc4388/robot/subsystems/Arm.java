package org.usfirst.frc4388.robot.subsystems;

import java.util.ArrayList;

import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.utility.CANTalonEncoder;
import org.usfirst.frc4388.utility.Loop;
import org.usfirst.frc4388.utility.MPTalonPIDController;
import org.usfirst.frc4388.utility.PIDParams;
import org.usfirst.frc4388.utility.TalonSRXEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem implements Loop
{
	private static Arm instance;

	public static enum ArmControlMode {PID, JOYSTICK_MANUAL };
	

	// One revolution of the 1-3 GEAR RATION ON THE ARM	* 4096 ticks 
	public static final double ENCODER_TICKS_TO_DEGREES = (36.0 / 12.0) * (36.0 / 24.0) * (34.0 / 24.0) * 4096.0 / (1.88 * Math.PI);   
	
	public double ARM_ANGLE_DEGREES = 0;

	// Defined speeds
	public static final double JOYSTICK_INCHES_PER_MS_HI = 0.75;
	
	// Defined positions
	public static final double ZERO_POSITION_AUTON_FORWARD_INCHES = 8.0;
	public static final double ZERO_POSITION_INCHES = -0.25;
	public static final double NEAR_ZERO_POSITION_INCHES = 0.0;
	public static final double MIN_POSITION_INCHES = 0.0;
	public static final double MAX_POSITION_INCHES = 83.4;
	public static final double AFTER_INTAKE_POSITION_INCHES = 4.0;


	// Motion profile max velocities and accel times
	public static final double MP_MAX_VELOCITY_INCHES_PER_SEC =  60; 
	public static final double MP_T1 = 400;  // Fast = 300
	public static final double MP_T2 = 150;  // Fast = 150
	
	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();	

	private CANTalonEncoder motor1;
	private TalonSRX motor2;
	
	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MP_SLOT = 1;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);  
	public static final double KF_UP = 0.005;
	public static final double KF_DOWN = 0.0;
	public static final double PID_ERROR_INCHES = 1.0;
	private long periodMs = (long)(Constants.kLooperDt * 1000.0);


	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;	
	private boolean isFinished;
	private ArmControlMode armControlMode = ArmControlMode.JOYSTICK_MANUAL;
	private double targetPositionInchesPID = 0;
	private boolean firstMpPoint;
	
	
	public Arm() {
		try {
			motor1 = new CANTalonEncoder(RobotMap.ARM_MOTOR1_ID, ENCODER_TICKS_TO_DEGREES, FeedbackDevice.QuadEncoder);
			//motor2 = CANTallon.createPermanentSlaveTalon(RobotMap.ARM_MOTOR_2_CAN_ID, RobotMap.ELEVATOR_MOTOR_1_CAN_ID);
		
			
			
		}
		catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

	//Set the degree to negative angle after initializing 
	public void setInitAngle()
	{
	  double armAngleToHoriz = 70;
	  double initAngle = ENCODER_TICKS_TO_DEGREES - armAngleToHoriz;
	  
	  ARM_ANGLE_DEGREES = initAngle;
	}

	@Override
	public void initDefaultCommand() {
	}
		
	public void resetZeroPosition(double position) {
		mpController.resetZeroPosition(position);
	}	
	
	private synchronized void setArmControlMode(ArmControlMode controlMode) {
		this.armControlMode = controlMode;
	}
	
	private synchronized ArmControlMode getArmControlMode() {
		return this.armControlMode;
	}

		
	public void setSpeedJoystick(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setArmControlMode(ArmControlMode.JOYSTICK_MANUAL);
	}
		
	public void setPositionPID(double targetPositionInches) {
		mpController.setPIDSlot(PID_SLOT);
		updatePositionPID(targetPositionInches);
		setArmControlMode(ArmControlMode.PID);	
		setFinished(false);
	}
	
	public void updatePositionPID(double targetPositionInches) {
 		targetPositionInchesPID = limitPosition(targetPositionInches);
		double startPositionInches = motor1.getPositionWorld();
		mpController.setTarget(targetPositionInchesPID, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN); 
	}

	
	private double limitPosition(double targetPosition) {
		if (targetPosition < MIN_POSITION_INCHES) {
			return MIN_POSITION_INCHES;
		}
		else if (targetPosition > MAX_POSITION_INCHES) {
			return MAX_POSITION_INCHES;
		}
		
		return targetPosition;
	}
	
	@Override
	public void onStart(double timestamp) {
		mpController.setPIDSlot(PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
	}

	@Override
	public void onStop(double timestamp) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (Arm.this) {
			switch( getArmControlMode() ) {
				case PID: 
					controlPidWithJoystick();
					break;
				case JOYSTICK_MANUAL:
					controlManualWithJoystick();
					break;
				default:
					break;
			}
		}
	}
	
	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition *.5;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
	}
	
	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick(joyStickSpeed);
	}
	

	public double getPositionInches() {
		return motor1.getPositionWorld();
	}
	
//	public double getAverageMotorCurrent() {
//		return (Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID)) / 3;
//	}
		
	public double getAverageMotorCurrent() {
		return (motor1.getOutputCurrent() + motor2.getOutputCurrent() / 2);
	}
		
	public synchronized boolean isFinished() {
		return isFinished;
	}
	
	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}
		
	public double getPeriodMs() {
		return periodMs;
	}
	
	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Elevator Position Inches", motor1.getPositionWorld());
				SmartDashboard.putNumber("Elevator Average Amps", getAverageMotorCurrent());
				SmartDashboard.putNumber("Elevator Target PID Position", targetPositionInchesPID);
			}
			catch (Exception e) {
			}
		}
	}	
	
	public static Arm getInstance() {
		if(instance == null) {
			instance = new Arm();
		}
		return instance;
	}
}

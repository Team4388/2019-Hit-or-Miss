package org.usfirst.frc4388.robot.subsystems;
import java.util.ArrayList;

import org.usfirst.frc4388.controller.XboxController;
import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.utility.Loop;
import org.usfirst.frc4388.utility.MPTalonPIDController;
import org.usfirst.frc4388.utility.PIDParams;
import org.usfirst.frc4388.utility.TalonSRXEncoder;
import org.usfirst.frc4388.utility.TalonSRXFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem implements Loop
{
	//PID encoder and motor
	private CANTalonEncoder armMotor;
	//private WPI_TalonSRX ArmLeft;

	public static enum ArmControlMode { MOTION_PROFILE, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL };

	// One revolution of the 30T Drive 1.880" PD pulley = Pi * PD inches = 36/24 revs due to pulleys * 34/24 revs due to gears * 36/12 revs due mag encoder gear on ball shifter * 4096 ticks 
	public static final double ENCODER_TICKS_TO_INCHES = (36.0 / 12.0) * (36.0 / 24.0) * (34.0 / 24.0) * 4096.0 / (1.88 * Math.PI);   
	
	// Defined speeds
	public static final double CLIMB_SPEED = -1.0;
	public static final double TEST_SPEED_UP = 0.3;
	public static final double TEST_SPEED_DOWN = -0.3;
	public static final double AUTO_ZERO_SPEED = -0.3;
	public static final double JOYSTICK_INCHES_PER_MS_HI = 0.75;
	public static final double JOYSTICK_INCHES_PER_MS_LO = JOYSTICK_INCHES_PER_MS_HI/3.68 * 0.8;
	
	// Defined positions
	public static final double ZERO_POSITION_AUTON_FORWARD_INCHES = 8.0;
	public static final double ZERO_POSITION_INCHES = -0.25;
	public static final double NEAR_ZERO_POSITION_INCHES = 0.0;
	public static final double MIN_POSITION_INCHES = 0.0;
	public static final double MAX_POSITION_INCHES = 83.4;
	public static final double AFTER_INTAKE_POSITION_INCHES = 4.0;

	public static final double SWITCH_POSITION_INCHES = 24.0;
	public static final double SWITCH_POSITION_HIGH_INCHES = 36.0; //Switch Position for First Cube APR
	public static final double SCALE_LOW_POSITION_INCHES = 56.0;
	public static final double SCALE_FIRST_CUBE_POSITION_INCHES = 78.0; //72.0
	public static final double SCALE_SECOND_CUBE_POSITION_INCHES = 77.0;
	public static final double SCALE_HIGH_POSITION_INCHES = MAX_POSITION_INCHES;
	public static final double CLIMB_BAR_POSITION_INCHES = 70.0;
	public static final double CLIMB_HIGH_POSITION_INCHES = 10.0;
	public static final double CLIMB_ASSIST_POSITION_INCHES = 50.0;

	// Motion profile max velocities and accel times
	public static final double MP_MAX_VELOCITY_INCHES_PER_SEC =  60; 
	public static final double MP_T1 = 400;  // Fast = 300
	public static final double MP_T2 = 150;  // Fast = 150
	
	//PID controller Max Scale
	private SoftwarePIDPositionController pidPositionControllerLowest;
	//private PIDParams PositionPIDParamsLowest = new PIDParams(2.0, 0.0, 0.0);
	private PIDParams PositionPLowest;

	private TalonSRX motor2;
	
	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MP_SLOT = 1;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);  
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);  
	public static final double KF_UP = 0.005;
	public static final double KF_DOWN = 0.0;
	public static final double PID_ERROR_INCHES = 1.0;
	private long periodMs = (long)(Constants.kLooperDt * 1000.0);

	// Pneumatics
	private Solenoid speedShift;

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;	
	private boolean isFinished;
	private ArmControlMode elevatorControlMode = ArmControlMode.JOYSTICK_MANUAL;
	private double targetPositionInchesPID = 0;
	private boolean firstMpPoint;
	private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
	
	private Arm() {
		try {
			motor1 = TalonSRXFactory.createTalonEncoder(RobotMap.ARM_MOTOR1_ID, ENCODER_TICKS_TO_INCHES, false, FeedbackDevice.QuadEncoder);
			motor2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ARM_MOTOR2_ID, RobotMap.ARM_MOTOR1_ID);
			
			
			motor1.setInverted(true);
			motor2.setInverted(true);
										
//	        if (motor1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            Driver.reportError("Could not detect elevator motor 1 encoder encoder!", false);
//	        }
			
			motorControllers.add(motor1);
			
			
		}
		firstMpPoint = true;
		setElevatorControlMode(ArmControlMode.MOTION_PROFILE);
 	}

    
	//PID encoder position
	public double getEncoderArmPosition()
	{
		return armMotor.getPositionWorld();
	}
	
	public double getArmHeightInchesAboveFloor()
	{
		return armMotor.getPositionWorld();
	}

	public synchronized void setControlMode(DriveControlMode controlMode) 
	{
 		this.controlMode = controlMode;
 		
 		isFinished = false;
	}
	/*
	public void setArmPIDMaxScale(double ArmPosition, double maxError, double minError)
	{
		double ArmTargetPos = 0;
		this.targetPPosition = ArmTargetPos;
		pidPositionControllerMaxScale.setPIDPositionTarget(ArmTargetPos, maxError, minError);      ///////TARGET POSITION WHERE??
		Robot.Arm.setControlMode(DriveControlMode.MOVE_POSITION_MAX_SCALE);
	}
	
	public void setArmPIDLowScale(double ArmPosition, double maxError, double minError)
	{
		double ArmTargetPos = 0;
		this.targetPPosition = ArmTargetPos;
		pidPositionControllerMaxScale.setPIDPositionTarget(ArmTargetPos, maxError, minError);
		Robot.Arm.setControlMode(DriveControlMode.MOVE_POSITION_LOW_SCALE);
	}
	
	public void setArmPIDSwitch(double ArmPosition, double maxError, double minError)
	{
		double ArmTargetPos = 0;
		this.targetPPosition = ArmTargetPos;
		pidPositionControllerMaxScale.setPIDPositionTarget(ArmTargetPos, maxError, minError);
		Robot.Arm.setControlMode(DriveControlMode.MOVE_POSITION_SWITCH);
	}
	
	@Override
	public void onStart(double timestamp) {
		mpController = new MPTalonPIDController(periodMs, motorControllers);
		mpController.setPID(mpPIDParams, MP_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
	}
	*/
	public void rawSetOutput(double output){
		armMotor.set(/*ControlMode.PercentOutput,*/ output);
	}

	@Override
	public void onLoop(double timestamp) {
		synchronized (Arm.this) {
			switch( getElevatorControlMode() ) {
				case JOYSTICK_PID: 
					controlPidWithJoystick();
					break;
				case JOYSTICK_MANUAL:
					controlManualWithJoystick();
					break;
				case MOTION_PROFILE: 
					if (!isFinished()) {
						if (firstMpPoint) {
							mpController.setPIDSlot(MP_SLOT);
							firstMpPoint = false;
						}
						setFinished(mpController.controlLoopUpdate()); 
						if (isFinished()) {
							mpController.setPIDSlot(PID_SLOT);
						}
					}
					break;
				default:
					break;
			}
			pressed = true;
		}
		else
		{
			if 	(controlMode == DriveControlMode.STOP_MOTORS){
				{
				Robot.arm.setControlMode(DriveControlMode.JOYSTICK);
				}
			
			pressed = false;
			}
		}

	}
		
		//pressed = (isPressed.isFwdLimitSwitchClosed() == true) ? true : false;
	
	
	
	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
	}
	
	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick(joyStickSpeed);
	}
	/*
	public void setShiftState(ElevatorSpeedShiftState state) {
		shiftState = state;
		if(state == ElevatorSpeedShiftState.HI) {
			joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_HI;
			speedShift.set(true);
			mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		}
		else if(state == ElevatorSpeedShiftState.LO) {
			joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
			speedShift.set(false);
			mpController.setPID(pidPIDParamsLoGear, PID_SLOT);
		}
	}
	
	public ElevatorSpeedShiftState getShiftState() {
		return shiftState;
	}
*/
	public double getPositionInches() {
		return motor1.getPositionWorld();
	}
	
//	public double getAverageMotorCurrent() {
//		return (Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID)) / 3;
//	}
		
	public double getAverageMotorCurrent() {
		return (motor1.getOutputCurrent() + motor2.getOutputCurrent()) / 2;
	}
		
	public synchronized boolean isFinished() {
		return isFinished;
	}
	
	public double getPeriodMs()
	{
		return periodMs;
	}
	
	public void updateStatus(Robot.OperationMode operationMode) {
		if (operationMode == Robot.OperationMode.TEST) {
			try {
				SmartDashboard.putNumber("Elevator Position Inches", motor1.getPositionWorld());
				SmartDashboard.putNumber("Elevator Motor 1 Amps", motor1.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Motor 2 Amps", motor2.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Average Amps", getAverageMotorCurrent());
//				SmartDashboard.putNumber("Elevator Motor 1 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 2 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 3 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID));
				SmartDashboard.putNumber("Elevator Target PID Position", targetPositionInchesPID);
			}
			catch (Exception e) 
			{
				System.err.println("Drivetrain update status error");
			}
		}
	}
}
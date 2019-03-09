package org.usfirst.frc4388.robot.subsystems;
import java.util.ArrayList;

import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.utility.ControlLoopable;
import org.usfirst.frc4388.utility.Loop;
import org.usfirst.frc4388.utility.MPTalonPIDController;
import org.usfirst.frc4388.utility.PIDParams;
import org.usfirst.frc4388.utility.TalonSRXEncoder;
import org.usfirst.frc4388.utility.TalonSRXFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Arm extends Subsystem implements ControlLoopable
{
	private static Arm instance;

	public static enum ArmControlMode { MOTION_PROFILE, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL };
	public static enum ArmPositionMode { CARGO, HATCH };

	// One revolution of the 30T Drive 1.880" PD pulley = Pi * PD inches = 36/24 revs due to pulleys * 34/24 revs due to gears * 36/12 revs due mag encoder gear on ball shifter * 4096 ticks 
	public static final double ENCODER_TICKS_TO_INCHES = (1);  
	

	private double periodMs;
	private double lastControlLoopUpdatePeriod = 0.0;
	private double lastControlLoopUpdateTimestamp = 0.0;
	// Defined speeds
	public static final double CLIMB_SPEED = -1.0;
	public static final double TEST_SPEED_UP = 0.3;
	public static final double TEST_SPEED_DOWN = -0.3;
	public static final double AUTO_ZERO_SPEED = -0.3;
	public static final double JOYSTICK_INCHES_PER_MS_HI = 20;
	public static final double JOYSTICK_INCHES_PER_MS_LO = 20;
	
	// Defined positions
	public static final double ZERO_POSITION_AUTON_FORWARD_INCHES = 8.0;
	public static final double ZERO_POSITION_INCHES = -0.25;
	public static final double NEAR_ZERO_POSITION_INCHES = 0.0;
	public static final double MIN_POSITION_INCHES = 0.0;
	public static final double MAX_POSITION_INCHES = 4096;
	public static final double AFTER_INTAKE_POSITION_INCHES = 4.0;

	public static final double HATCH_LOW_POSITION_WORLD = 1000;
	public static final double HATCH_MID_POSITION_WORLD = 2500;
	public static final double HATCH_HIGH_POSITION_WORLD = 3800;
	public static final double CARGO_LOW_POSITION_WORLD = 1500;
	public static final double CARGO_MID_POSITION_WORLD = 2900;
	public static final double CARGO_HIGH_POSITION_WORLD = 4300;
	public static final double CARGO_PICKUP_POSITION_WORLD = 0;
	public static final double HATCH_PICKUP_POSITION_WORLD = 0;
 
	/*public static final double SWITCH_POSITION_INCHES = 24.0;
	public static final double SWITCH_POSITION_HIGH_INCHES = 36.0; //Switch Position for First Cube APR
	public static final double SCALE_LOW_POSITION_INCHES = 56.0;
	public static final double SCALE_FIRST_CUBE_POSITION_INCHES = 78.0; //72.0
	public static final double SCALE_SECOND_CUBE_POSITION_INCHES = 77.0;
	public static final double SCALE_HIGH_POSITION_INCHES = MAX_POSITION_INCHES;
	public static final double CLIMB_BAR_POSITION_INCHES = 70.0;
	public static final double CLIMB_HIGH_POSITION_INCHES = 10.0;
	public static final double CLIMB_ASSIST_POSITION_INCHES = 50.0;*/

	// Motion profile max velocities and accel times
	public static final double MP_MAX_VELOCITY_INCHES_PER_SEC =  60; 
	public static final double MP_T1 = 400;  // Fast = 300
	public static final double MP_T2 = 150;  // Fast = 150
	
	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();	

	private TalonSRXEncoder motor1;
	private TalonSRX motor2;
	
	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MP_SLOT = 1;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);  
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);  
	public static final double KF_UP = 0.01;
	public static final double KF_DOWN = 0.0;
	public static final double P_Value = 2;
	public static final double I_Value = 0.00300;
	public static final double D_Value = 200;
	public static final double RampRate = 0.0;
	private PIDParams armPIDParams = new PIDParams(P_Value, I_Value, D_Value, KF_DOWN);	// KF gets updated later
	public static final double PID_ERROR_INCHES = 5.0;
	LimitSwitchSource limitSwitchSource;
	DigitalInput forwardLimitSwitch = new DigitalInput(1);
	DigitalInput reverseLimitSwitch = new DigitalInput(2);
	
	// Pneumatics
	private Solenoid speedShift;

	//DPAD
	public static final double DPAD_UP = 0;
	public static final double DPAD_RIGHT = 90;
	public static final double DPAD_DOWN = 180;
	public static final double DPAD_LEFT = 270;
	public static final double DPAD_RELEASED = -1;

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;	
	private boolean isFinished;
	private ArmControlMode armControlMode = ArmControlMode.JOYSTICK_MANUAL;
	public ArmPositionMode armPositionMode = ArmPositionMode.HATCH;
	private double targetPositionInchesPID = 0;
	private boolean firstMpPoint;
	private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
	private double p = 0;

	
	public Arm() {
		try {
			motor1 = TalonSRXFactory.createTalonEncoder(RobotMap.ARM_MOTOR1_ID, (ENCODER_TICKS_TO_INCHES), false, FeedbackDevice.QuadEncoder);
			motor2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ARM_MOTOR2_ID, RobotMap.ARM_MOTOR1_ID);
			
			
			motor1.setInverted(true);
			motor2.setInverted(true);
									
//	        if (motor1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            Driver.reportError("Could not detect elevator motor 1 encoder encoder!", false);
//	        }
			
			motor1.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			motor1.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			motor1.setNeutralMode(NeutralMode.Brake);
    		motor2.setNeutralMode(NeutralMode.Brake);
			motor1.enableCurrentLimit(true);
			motorControllers.add(motor1);
			
			
		}
		catch (Exception e) {
			System.err.println("An error occurred in the Arm constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}
		
	public void resetZeroPosition(double position) {
		mpController.resetZeroPosition(position);
	}	
	public void resetencoder(){
		motor1.setPosition(0);
	}
	
	private synchronized void setArmControlMode(ArmControlMode controlMode) {
		this.armControlMode = controlMode;
	}
	
	private synchronized ArmControlMode getArmControlMode() {
		return this.armControlMode;
	}

	public synchronized void setArmPositionMode(ArmPositionMode controlMode) {
		this.armPositionMode = controlMode;
	}
	
	private synchronized ArmPositionMode getArmPositionMode() {
		return this.armPositionMode;
	}

	public void setSpeed(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setArmControlMode(ArmControlMode.MANUAL);
	}
		
	public void setSpeedJoystick(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setArmControlMode(ArmControlMode.JOYSTICK_MANUAL);
	}
		
	public void setPositionPID(double targetPositionInches) {
		motor1.set(ControlMode.Position, targetPositionInches);
		mpController.setPIDSlot(PID_SLOT);	//TODO: verify that motor's selectProfileSlot() should be called AFTER its control mode is set
		updatePositionPID(targetPositionInches);
		setArmControlMode(ArmControlMode.JOYSTICK_PID);	
		setFinished(false);
	}
	
	public void updatePositionPID(double targetPositionInches) {
 		targetPositionInchesPID = limitPosition(targetPositionInches);
		double startPositionInches = motor1.getPositionWorld();
		//mpController.setTarget(targetPositionInchesPID, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN); 
		motor1.set(ControlMode.Position, targetPositionInches);
		motor1.configClosedloopRamp(.02);
		//motor1.configPeakCurrentLimit(5);
		motor1.configContinuousCurrentLimit(2);
		motor1.config_kP(0, P_Value, TalonSRXEncoder.TIMEOUT_MS);
		motor1.config_kI(0, I_Value, TalonSRXEncoder.TIMEOUT_MS);
		motor1.config_kD(0, D_Value, TalonSRXEncoder.TIMEOUT_MS);
		motor1.config_kF(0, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN, TalonSRXEncoder.TIMEOUT_MS);
		//System.err.println(motor1.getControlMode());
		//System.err.print(motor1.getClosedLoopError());
	}
	
	public void setPositionMP(double targetPositionInches) {
		double startPositionInches = motor1.getPositionWorld();
		mpController.setMPTarget(startPositionInches, limitPosition(targetPositionInches), MP_MAX_VELOCITY_INCHES_PER_SEC, MP_T1, MP_T2); 
		setFinished(false);
		firstMpPoint = true;
		setArmControlMode(ArmControlMode.MOTION_PROFILE);
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
	public void setPeriodMs(long periodMs) {
		mpController = new MPTalonPIDController(periodMs, motorControllers);
		mpController.setPID(mpPIDParams, MP_SLOT);
		mpController.setPID(armPIDParams, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
		this.periodMs = periodMs;
	}
	/*@Override
	public void onStart(double timestamp) {
		mpController = new MPTalonPIDController(periodMs, motorControllers);
		mpController.setPID(mpPIDParams, MP_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
		mpController.setPID(pidPIDParamsHiGear, PID_SLOT);
		mpController.setPIDSlot(PID_SLOT);
	}

	@Override
	public void onStop(double timestamp) {
		// TODO Auto-generated method stub
		
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
		}
	}

*/





	public synchronized void controlLoopUpdate() {
		// Measure *actual* update period
		double currentTimestamp = Timer.getFPGATimestamp();
		if (lastControlLoopUpdateTimestamp > 0.001)	{	// ie, if this is NOT the first time
			lastControlLoopUpdatePeriod = currentTimestamp - lastControlLoopUpdateTimestamp;
		}
		lastControlLoopUpdateTimestamp = currentTimestamp;
		
		if (reverseLimitSwitch.get()){
			motor1.setPosition(0);
		}

		// Do the update
		if (armControlMode == ArmControlMode.JOYSTICK_MANUAL) {
			controlManualWithJoystick();
		}
		else if (!isFinished) {
			if (armControlMode == ArmControlMode.MOTION_PROFILE) {
				isFinished = mpController.controlLoopUpdate(getPositionInches()); 
				
			}
			if (armControlMode == ArmControlMode.JOYSTICK_PID){
				//System.err.println(motor1.getControlMode());
				int dPadAngle = Robot.oi.getOperatorController().getDpadAngle();
				if (dPadAngle == DPAD_RELEASED){
					controlPidWithJoystick();
				} else {
					controlPidWithDPad(dPadAngle);
				}
			}
			
			/*else if (armControlMode == ArmControlMode.MP_PATH_VELOCITY) {
				isFinished = mpPathVelocityController.controlLoopUpdate(getGyroAngleDeg()); 
			}
			else if (armControlMode == ArmControlMode.ADAPTIVE_PURSUIT) {
				updatePose();
				isFinished = adaptivePursuitController.controlLoopUpdate(currentPose); 
			}*/
		}
	}

	private void controlPidWithDPad(int dPadAngle){
		if (armPositionMode == ArmPositionMode.HATCH){
			if (dPadAngle == DPAD_UP){
				setPositionPID(HATCH_HIGH_POSITION_WORLD);
			} else if (dPadAngle == DPAD_RIGHT){
				setPositionPID(HATCH_MID_POSITION_WORLD);
			} else if (dPadAngle == DPAD_DOWN){
				setPositionPID(HATCH_LOW_POSITION_WORLD);
			} else if (dPadAngle == DPAD_LEFT){
				//setPositionPID(HATCH_PICKUP_POSITION_WORLD);
			}
		} else if (armPositionMode == ArmPositionMode.CARGO){
			if (dPadAngle == DPAD_UP){
				setPositionPID(CARGO_HIGH_POSITION_WORLD);
			} else if (dPadAngle == DPAD_RIGHT){
				setPositionPID(CARGO_MID_POSITION_WORLD);
			} else if (dPadAngle == DPAD_DOWN){
				setPositionPID(CARGO_LOW_POSITION_WORLD);
			} else if (dPadAngle == DPAD_LEFT){
				//setPositionPID(CARGO_PICKUP_POSITION_WORLD);
			}
		}
	}
	
	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
		
		
	}

	private void ControlWithJoystickhold(){
		double holdPosition = motor1.getPositionWorld();
		updatePositionPID(holdPosition);

	}
	
	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick((joyStickSpeed*.30)+.1);
	}
	/*
	public void setShiftState(ElevatorSpeedShiftState state) {

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
	
	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}
		
	public double getPeriodMs() {
		return periodMs;
	}
	
	public void updateStatus(Robot.OperationMode operationMode) {
		//System.err.println("the encoder is right after this");
			try {
				
				SmartDashboard.putNumber("Arm Position", motor1.getPositionWorld());
				SmartDashboard.putNumber("Arm Motor 1 Amps", motor1.getOutputCurrent());
				SmartDashboard.putNumber("Arm Motor 2 Amps", motor2.getOutputCurrent());
				SmartDashboard.putNumber("Arm Average Amps", getAverageMotorCurrent());
				SmartDashboard.putNumber("arm pid error", motor1.getClosedLoopError());
				SmartDashboard.putNumber("arm motor output", motor1.getMotorOutputPercent());
				
//				SmartDashboard.putNumber("Elevator Motor 1 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 2 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 3 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID));
				SmartDashboard.putNumber("Arm Target PID Position", targetPositionInchesPID);
			}
			catch (Exception e) {
				System.err.println("Drivetrain update status error" +e.getMessage());
			}
		
	}	
	
	public static Arm getInstance() {
		if(instance == null) {
			instance = new Arm();
		}
		return instance;
	}
}

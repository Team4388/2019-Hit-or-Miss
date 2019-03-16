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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Wrist extends Subsystem implements ControlLoopable
{
	private static Wrist instance;

	public static enum WristControlMode { MOTION_PROFILE, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL };

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
	
	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();	

	private TalonSRXEncoder wristMotor1;
	
	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MP_SLOT = 1;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);  
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);  
	public static final double KF_UP = 0.01;
	public static final double KF_DOWN = 0.0;
	public static final double P_Value = 3;
	public static final double I_Value = 0.001;
	public static final double D_Value = 250;
	public static final double RampRate = 0.0;
	private PIDParams wristPIDParams = new PIDParams(P_Value, I_Value, D_Value, KF_DOWN);	// KF gets updated later
	public static final double PID_ERROR_INCHES = 150;
	LimitSwitchSource limitSwitchSource;
	// Pneumatics
	private Solenoid speedShift;

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;	
	private boolean isFinished;
	private WristControlMode wristControlMode = WristControlMode.JOYSTICK_PID;
	public static double targetPositionInchesPID = 0;
	private boolean firstMpPoint;
	private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
	private double p = 0;

	
	public Wrist() {
		try {
			wristMotor1 = TalonSRXFactory.createTalonEncoder(RobotMap.WRIST_MOTOR_ID, (ENCODER_TICKS_TO_INCHES), false, FeedbackDevice.QuadEncoder);	
			wristMotor1.setInverted(false);

									
//	        if (wristMotor1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
//	            Driver.reportError("Could not detect elevator motor 1 encoder encoder!", false);
//	        }
			
			wristMotor1.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			wristMotor1.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			wristMotor1.setNeutralMode(NeutralMode.Brake);
			wristMotor1.enableCurrentLimit(true);
			//wristMotor1.setSensorPhase(true);
			motorControllers.add(wristMotor1);
			
			
		}
		catch (Exception e) {
			System.err.println("An error occurred in the Wrist constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}
		
	public void resetZeroPosition(double position) {
		mpController.resetZeroPosition(position);
	}	
	public void resetencoder(){
		wristMotor1.setPosition(0);
	}
	
	
	private synchronized void setWristControlMode(WristControlMode controlMode) {
		this.wristControlMode = controlMode;
	}
	
	private synchronized WristControlMode getWristControlMode() {
		return this.wristControlMode;
	}

	public void setSpeed(double speed) {
		wristMotor1.set(ControlMode.PercentOutput, speed);
		setWristControlMode(WristControlMode.MANUAL);
	}
		
	public void setSpeedJoystick(double speed) {
		wristMotor1.set(ControlMode.PercentOutput, speed);
		setWristControlMode(WristControlMode.JOYSTICK_MANUAL);
	}
		
	public void setPositionPID(double targetPositionInches) {
		wristMotor1.set(ControlMode.Position, targetPositionInches);
		mpController.setPIDSlot(PID_SLOT);	//TODO: verify that motor's selectProfileSlot() should be called AFTER its control mode is set
		updatePositionPID(targetPositionInches);
		setWristControlMode(WristControlMode.JOYSTICK_PID);	
		setFinished(false);
	}
	
	public void updatePositionPID(double targetPositionInches) {
 		targetPositionInchesPID = limitPosition(targetPositionInches);
		double startPositionInches = wristMotor1.getPositionWorld();
		//mpController.setTarget(targetPositionInchesPID, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN); 
		wristMotor1.set(ControlMode.Position, targetPositionInches);
		wristMotor1.configClosedloopRamp(.02);
		//wristMotor1.configPeakCurrentLimit(5);
		wristMotor1.configContinuousCurrentLimit(2);
		wristMotor1.config_kP(0, P_Value, TalonSRXEncoder.TIMEOUT_MS);
		wristMotor1.config_kI(0, I_Value, TalonSRXEncoder.TIMEOUT_MS);
		wristMotor1.config_kD(0, D_Value, TalonSRXEncoder.TIMEOUT_MS);
		wristMotor1.config_kF(0, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN, TalonSRXEncoder.TIMEOUT_MS);
		//System.err.println(wristMotor1.getControlMode());
		//System.err.print(wristMotor1.getClosedLoopError());
	}
	
	public void setPositionMP(double targetPositionInches) {
		double startPositionInches = wristMotor1.getPositionWorld();
		mpController.setMPTarget(startPositionInches, limitPosition(targetPositionInches), MP_MAX_VELOCITY_INCHES_PER_SEC, MP_T1, MP_T2); 
		setFinished(false);
		firstMpPoint = true;
		setWristControlMode(WristControlMode.MOTION_PROFILE);
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
		mpController.setPID(wristPIDParams, PID_SLOT);
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
		synchronized (Wrist.this) {
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
		
		// Do the update
		if (wristControlMode == WristControlMode.JOYSTICK_MANUAL) {
			controlManualWithJoystick();
			
		}
		else if (!isFinished) {
			if (wristControlMode == WristControlMode.MOTION_PROFILE) {
				isFinished = mpController.controlLoopUpdate(getPositionInches()); 
				
			}
			if (wristControlMode == WristControlMode.JOYSTICK_PID){
				System.err.println(wristMotor1.getControlMode());
				controlPidWithJoystick();
				
	
			}
			
			/*else if (wristControlMode == WristControlMode.MP_PATH_VELOCITY) {
				isFinished = mpPathVelocityController.controlLoopUpdate(getGyroAngleDeg()); 
			}
			else if (wristControlMode == WristControlMode.ADAPTIVE_PURSUIT) {
				updatePose();
				isFinished = adaptivePursuitController.controlLoopUpdate(currentPose); 
			}*/
		}
	}






	
	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getRightYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
		
		
	}

	private void ControlWithJoystickhold(){
		double holdPosition = wristMotor1.getPositionWorld();
		updatePositionPID(holdPosition);

	}
	
	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getRightYAxis();
		setSpeedJoystick(joyStickSpeed*.5);
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
		return wristMotor1.getPositionWorld();
	}
	
//	public double getAverageMotorCurrent() {
//		return (Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID) + Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID)) / 3;
//	}
		

		
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
				
				SmartDashboard.putNumber("Wrist Position Ticks", wristMotor1.getPositionWorld());
				SmartDashboard.putNumber("Wrist Motor 1 Amps", wristMotor1.getOutputCurrent());
				SmartDashboard.putNumber("wrist pid error", wristMotor1.getClosedLoopError());
				SmartDashboard.putNumber("wrist motor output", wristMotor1.getMotorOutputPercent());
				
//				SmartDashboard.putNumber("Elevator Motor 1 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 2 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 3 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID));
				SmartDashboard.putNumber("Wrist Target PID Position", targetPositionInchesPID);
			}
			catch (Exception e) {
				System.err.println("Drivetrain update status error" +e.getMessage());
			}
		
	}	
	
	public static Wrist getInstance() {
		if(instance == null) {
			instance = new Wrist();
		}
		return instance;
	}
}

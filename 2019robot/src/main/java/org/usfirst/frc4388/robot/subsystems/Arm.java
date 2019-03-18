package org.usfirst.frc4388.robot.subsystems;
import java.util.ArrayList;

import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.robot.commands.ArmSetPositionMM;
import org.usfirst.frc4388.robot.commands.SetPositionArmWrist;
import org.usfirst.frc4388.robot.commands.StowArm;
import org.usfirst.frc4388.utility.ControlLoopable;
import org.usfirst.frc4388.utility.Loop;
import org.usfirst.frc4388.utility.MPTalonPIDController;
import org.usfirst.frc4388.utility.PIDParams;
import org.usfirst.frc4388.utility.TalonSRXEncoder;
import org.usfirst.frc4388.utility.TalonSRXFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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

	public static enum ArmControlMode { MOTION_PROFILE, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL, MOTION_MAGIC};
	public static enum PlaceMode { HATCH, CARGO };

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
	public static final double JOYSTICK_INCHES_PER_MS_HI = 35;
	public static final double JOYSTICK_INCHES_PER_MS_LO = 35;

	// Defined positions
	public static final double ZERO_POSITION_AUTON_FORWARD_INCHES = 8.0;
	public static final double ZERO_POSITION_INCHES = -0.25;
	public static final double NEAR_ZERO_POSITION_INCHES = 0.0;
	public static final double MIN_POSITION_INCHES = -25;
	public static final double MAX_POSITION_INCHES = 4400;
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
	public static final double MP_T1 = 300;  // Fast = 300
	public static final double MP_T2 = 150;  // Fast = 150

	// Motor controllers
	private ArrayList<TalonSRXEncoder> motorControllers = new ArrayList<TalonSRXEncoder>();

	public TalonSRXEncoder motor1;
	private TalonSRX motor2;

	// PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MM_SLOT = 1;
	public static int MP_SLOT = 2;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);
	public static final double KF_UP = 1;//0.01;
	public static final double KF_DOWN = 0;// 0.0;
	public static final double P_Value = 0.5;// 2;
	public static final double I_Value = 0.0008;// 0.00030;
	public static final double D_Value = 100;// 200;
	public static final double F_Value = 0.75;	// 1023 / 1360 max speed (ticks/100ms)
	public static final double maxGravityComp = 0.08;
	public static final double RampRate = 0;// 0.0;
	public static final int A_value = 450;
	public static final int CV_value = 740;




	private PIDParams armPIDParams = new PIDParams(P_Value, I_Value, D_Value, KF_DOWN);	// KF gets updated later
	public static final double PID_ERROR_INCHES = 5;
	LimitSwitchSource limitSwitchSource;

	// Pneumatics
	private Solenoid speedShift;

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;
	private boolean isFinished;
	private ArmControlMode armControlMode = ArmControlMode.MOTION_MAGIC;
	public PlaceMode placeMode = PlaceMode.HATCH;
	public double targetPositionInchesPID = 0;
	public double targetPositionInchesMM = 0;
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

			motor1.configNominalOutputForward(0, TalonSRXEncoder.TIMEOUT_MS);
			motor1.configNominalOutputReverse(0, TalonSRXEncoder.TIMEOUT_MS);
			motor1.configPeakOutputForward(1, TalonSRXEncoder.TIMEOUT_MS);
			motor1.configPeakOutputReverse(-1, TalonSRXEncoder.TIMEOUT_MS);

			motor1.selectProfileSlot(MM_SLOT, 0);
			motor1.config_kF(MM_SLOT, F_Value, TalonSRXEncoder.TIMEOUT_MS);
			motor1.config_kP(MM_SLOT, P_Value, TalonSRXEncoder.TIMEOUT_MS);
			motor1.config_kI(MM_SLOT, I_Value, TalonSRXEncoder.TIMEOUT_MS);
			motor1.config_kD(MM_SLOT, D_Value, TalonSRXEncoder.TIMEOUT_MS);

			motor1.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			motor1.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			motor1.setNeutralMode(NeutralMode.Brake);
    		motor2.setNeutralMode(NeutralMode.Brake);
			motor1.enableCurrentLimit(true);
			motorControllers.add(motor1);
			//motor1.setSelectedSensorPosition(0, , );


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

	public void resetEncoder(){
		motor1.setPosition(0);
		targetPositionInchesMM = 0;
		targetPositionInchesPID = 0;
	}

	private synchronized void setArmControlMode(ArmControlMode controlMode) {
		this.armControlMode = controlMode;
	}

	private synchronized ArmControlMode getArmControlMode() {
		return this.armControlMode;
	}

	public void setSpeed(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setArmControlMode(ArmControlMode.MANUAL);
	}

	public void setSpeedJoystick(double speed) {
		motor1.set(ControlMode.PercentOutput, speed);
		setArmControlMode(ArmControlMode.JOYSTICK_MANUAL);
	}
	public void setPositionMM(double targetPositionInches){
		motor1.set(ControlMode.MotionMagic, targetPositionInches);
		System.err.println(motor1.getControlMode());
		motor1.selectProfileSlot(MM_SLOT, 0);
		setArmControlMode(ArmControlMode.MOTION_MAGIC);
		updatePositionMM(targetPositionInches);
		setFinished(false);
	}

	public double calcGravityCompensationAtCurrentPosition() {
		int ticks = motor1.getSelectedSensorPosition();
		double degreesFromDown = (ticks+920)*(360.0/(4096*3));
		double compensation = maxGravityComp * Math.sin(degreesFromDown*Math.PI/180);
		System.err.println("comp(" + degreesFromDown + "^) = " + compensation);
		return compensation;
	}
	public void updatePositionMM(double targetPositionInches){
		targetPositionInchesMM = limitPosition(targetPositionInches);
		//double startPositionInches = motor1.getPositionWorld();
		double compensation = calcGravityCompensationAtCurrentPosition();
		//System.err.println("compensation = " + compensation);
		// motor1.set(ControlMode.MotionMagic, targetPositionInches);
		motor1.set(ControlMode.MotionMagic, targetPositionInches, DemandType.ArbitraryFeedForward, compensation);
		//System.err.println(motor1.getControlMode());
		motor1.configMotionCruiseVelocity(CV_value, TalonSRXEncoder.TIMEOUT_MS);
		motor1.configMotionAcceleration(A_value, TalonSRXEncoder.TIMEOUT_MS);


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
		if (limitPosition(motor1.getPositionWorld()) == MIN_POSITION_INCHES){
			resetEncoder();
		}
		double startPositionInches = motor1.getPositionWorld();
		//mpController.setTarget(targetPositionInchesPID, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN);
		motor1.set(ControlMode.Position, targetPositionInches);
		motor1.configClosedloopRamp(0);
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
		/*if (targetPosition < MIN_POSITION_INCHES) {
			return MIN_POSITION_INCHES;
		}*/
		if (targetPosition > MAX_POSITION_INCHES) {
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

	private int lastDPadAngle = -1;
	public void dPadButtons(){
		int dPadAngle = Robot.oi.getOperatorController().getDpadAngle();
		if (placeMode == PlaceMode.HATCH){
			if (dPadAngle == 0 && lastDPadAngle == -1){
				new SetPositionArmWrist(3605, 1144).start();
			}
			if (dPadAngle == 90 && lastDPadAngle == -1){
				new SetPositionArmWrist(2000, 750).start();
			}
			if (dPadAngle == 180 && lastDPadAngle == -1){
				new SetPositionArmWrist(590, 450).start();
			}
		}
		if (placeMode == PlaceMode.CARGO) {
			if (dPadAngle == 0 && lastDPadAngle == -1){
				new SetPositionArmWrist(4298, 3243).start();
			}
			if (dPadAngle == 90 && lastDPadAngle == -1){
				new SetPositionArmWrist(2830, 2830).start();
			}
			if (dPadAngle == 180 && lastDPadAngle == -1){
				new SetPositionArmWrist(1388, 2500).start();
			}
		}

		if (dPadAngle == 270 && lastDPadAngle == -1){
			new StowArm().start();
		}
		SmartDashboard.putNumber("DPad Angle", dPadAngle);
		lastDPadAngle = dPadAngle;
	}

	public synchronized void controlLoopUpdate() {
		// Measure *actual* update period
		double currentTimestamp = Timer.getFPGATimestamp();
		if (lastControlLoopUpdateTimestamp > 0.001)	{	// ie, if this is NOT the first time
			lastControlLoopUpdatePeriod = currentTimestamp - lastControlLoopUpdateTimestamp;
		}
		lastControlLoopUpdateTimestamp = currentTimestamp;

		dPadButtons();

		if (motor1.getSensorCollection().isRevLimitSwitchClosed()){
			resetEncoder();
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
					controlPidWithJoystick();
					System.err.println(motor1.getControlMode());
			}
			if (armControlMode == ArmControlMode.MOTION_MAGIC){
				controlMMWithJoystick();
				//System.err.println(motor1.getControlMode());
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







	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
	}
	private void controlMMWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesMM = targetPositionInchesMM + deltaPosition;
		updatePositionMM(targetPositionInchesMM);
		//Robot.wrist.targetPositionInchesPID = targetPositionInchesPID - (deltaPosition/3);
		Robot.wrist.updatePositionPID(Robot.wrist.targetPositionInchesPID);


	}

	private void ControlWithJoystickhold(){
		double holdPosition = motor1.getPositionWorld();
		updatePositionPID(holdPosition);

	}

	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick((joyStickSpeed*.3)+.08);
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

				SmartDashboard.putNumber("Arm Position Ticks", motor1.getPositionWorld());
				//SmartDashboard.putNumber("Arm Motor 1 Amps", motor1.getOutputCurrent());
				//SmartDashboard.putNumber("Arm Motor 2 Amps", motor2.getOutputCurrent());
				SmartDashboard.putNumber("sensor vel", motor1.getSelectedSensorVelocity());
				SmartDashboard.putNumber("Arm Average Amps", getAverageMotorCurrent());
				SmartDashboard.putNumber("arm pid error", motor1.getClosedLoopError());
				SmartDashboard.putNumber("arm motor output", motor1.getMotorOutputPercent());
				SmartDashboard.putNumber("Arm Target MM Position", targetPositionInchesMM);
				//SmartDashboard.putNumber("arm output", motor1.getMotorOutputPercent());
//				SmartDashboard.putNumber("Elevator Motor 1 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 2 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID));
//				SmartDashboard.putNumber("Elevator Motor 3 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID));
				SmartDashboard.putNumber("Arm Target PID Position", targetPositionInchesPID);
			}
			catch (Exception e) {
				System.err.println("Arm update status error" +e.getMessage());
			}

	}

	public static Arm getInstance() {
		if(instance == null) {
			instance = new Arm();
		}
		return instance;
	}
}

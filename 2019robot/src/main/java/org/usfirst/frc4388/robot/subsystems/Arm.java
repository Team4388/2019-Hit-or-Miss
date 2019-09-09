package org.usfirst.frc4388.robot.subsystems;

import java.util.ArrayList;

import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.robot.commands.ArmSetPositionSM;
import org.usfirst.frc4388.robot.commands.presets.CargoHigh;
import org.usfirst.frc4388.robot.commands.presets.CargoLow;
import org.usfirst.frc4388.robot.commands.presets.CargoMid;
import org.usfirst.frc4388.robot.commands.presets.HatchHigh;
import org.usfirst.frc4388.robot.commands.presets.HatchLow;
import org.usfirst.frc4388.robot.commands.presets.HatchMid;
import org.usfirst.frc4388.robot.commands.presets.SetPositionArmWrist;
import org.usfirst.frc4388.robot.commands.presets.StowArm;
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
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

/**
 * Controls a single joint arm. Uses a joystick to change a MM or PID target position for the arm,
 * and uses the d-pad on the opperator controller to move the arm and wrist to preset positions.
 */
public class Arm extends Subsystem implements ControlLoopable
{
	private static Arm instance;

	public static enum ArmControlMode { MOTION_PROFILE, JOYSTICK_PID, JOYSTICK_MANUAL, MANUAL, MOTION_MAGIC, SMART_MOTION};
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
	public static final double MAX_POSITION_INCHES = 4200;
	public static final double AFTER_INTAKE_POSITION_INCHES = 4.0;


	// Motion profile max velocities and accel times
	//public static final double MP_MAX_VELOCITY_INCHES_PER_SEC =  60;
	//public static final double MP_T1 = 300;  // Fast = 300
	//public static final double MP_T2 = 150;  // Fast = 150

	// Motor controllers
	public CANSparkMax motor1;
	private CANSparkMax motor2;

	private CANPIDController motorController;
	private CANEncoder motorEncoder;
	private CANDigitalInput motorForwardLimitSwitch, motorReverseLimitSwitch;

	// PID controller and params
	//private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	//public static int MM_SLOT = 1;
	//public static int MP_SLOT = 2;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);
	public static double kFF_Up = 1;//0.01;
	public static double kFF_Down = 0;// 0.0;
	public static double kP = 4;// 2;
	public static double kI = 0.0001;// 0.00030;
	public static double kD = 200;// 200;
	public static double kIz = 0;
	public static double kFF = 0.75;	// 1023 / 1360 max speed (ticks/100ms)
	public static double maxGravityComp = 0.01;
	public static double RampRate = 0;// 0.0;
	public static int maxAcc = 400;
	public static int maxVel = 500;
	public static double kMinOutput = -1;
	public static double kMaxOutput = 1;

	private PIDParams armPIDParams = new PIDParams(kP, kI, kD, kFF_Down);	// KF gets updated later
	public static final double PID_ERROR_INCHES = 50;
	LimitSwitchSource limitSwitchSource;

	// Pneumatics
	private Solenoid speedShift;

	//DPad Buttons
	static final int DPAD_UP = 0;
	static final int DPAD_RIGHT = 90;
	static final int DPAD_DOWN = 180;
	static final int DPAD_LEFT = 270;

	// Misc
	public static final double AUTO_ZERO_MOTOR_CURRENT = 4.0;
	private boolean isFinished;
	private ArmControlMode armControlMode = ArmControlMode.SMART_MOTION;
	public PlaceMode placeMode = PlaceMode.HATCH;
	public double targetPositionInchesPID = 0;
	public double targetPositionInchesSM = 0;
	//public double targetPositionInchesMM = 0;
	//private boolean firstMpPoint;
	private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
	private double p = 0;


	public Arm() {
		try {
			// motor1 = TalonSRXFactory.createTalonEncoder(RobotMap.ARM_MOTOR1_ID, (ENCODER_TICKS_TO_INCHES), false, FeedbackDevice.QuadEncoder);
			// motor2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.ARM_MOTOR2_ID, RobotMap.ARM_MOTOR1_ID);

			motor1 = new CANSparkMax(RobotMap.ARM_MOTOR1_ID, MotorType.kBrushless);
			motor2 = new CANSparkMax(RobotMap.ARM_MOTOR2_ID, MotorType.kBrushless);
			motor1.restoreFactoryDefaults();
			motor2.restoreFactoryDefaults();
			motor2.follow(motor1);
			motorController = motor1.getPIDController();
			motorEncoder = motor1.getEncoder();
			motorForwardLimitSwitch = motor1.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
			motorReverseLimitSwitch = motor1.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

			motorController.setP(kP);
			motorController.setI(kI);
    		motorController.setD(kD);
    		motorController.setIZone(kIz);
			motorController.setFF(kFF);
			motorController.setOutputRange(kMinOutput, kMaxOutput);


			// motor1.setInverted(true);
			// motor2.setInverted(true);
			// if (motor1.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
			//    Driver.reportError("Could not detect elevator motor 1 encoder encoder!", false);
			// }
			// motor1.configNominalOutputForward(0, TalonSRXEncoder.TIMEOUT_MS);
			// motor1.configNominalOutputReverse(0, TalonSRXEncoder.TIMEOUT_MS);
			// motor1.configPeakOutputForward(1, TalonSRXEncoder.TIMEOUT_MS);
			// motor1.configPeakOutputReverse(-1, TalonSRXEncoder.TIMEOUT_MS);
			// motor1.selectProfileSlot(MM_SLOT, 0);
			// motor1.config_kF(MM_SLOT, F_Value, TalonSRXEncoder.TIMEOUT_MS);
			// motor1.config_kP(MM_SLOT, P_Value, TalonSRXEncoder.TIMEOUT_MS);
			// motor1.config_kI(MM_SLOT, I_Value, TalonSRXEncoder.TIMEOUT_MS);
			// motor1.config_kD(MM_SLOT, D_Value, TalonSRXEncoder.TIMEOUT_MS);
			// motor1.setSensorPhase(true);
			// motor1.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			// motor1.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
			// motor1.setNeutralMode(NeutralMode.Brake);
    		// motor2.setNeutralMode(NeutralMode.Brake);
			// motor1.enableCurrentLimit(true);
			// motorControllers.add(motor1);
			//motor1.setSelectedSensorPosition(0, , );

		}
		catch (Exception e) {
			System.err.println("An error occurred in the Arm constructor");
		}
	}

	@Override
	public void initDefaultCommand() {
	}

	public synchronized void controlLoopUpdate() {
		// Measure *actual* update period
		double currentTimestamp = Timer.getFPGATimestamp();
		if (lastControlLoopUpdateTimestamp > 0.001)	{	// ie, if this is NOT the first time
			lastControlLoopUpdatePeriod = currentTimestamp - lastControlLoopUpdateTimestamp;
		}
		lastControlLoopUpdateTimestamp = currentTimestamp;

		dPadButtons();

		if (motorReverseLimitSwitch.get()){
			resetEncoder();
		}

		// Do the update
		if (armControlMode == ArmControlMode.JOYSTICK_MANUAL) {
			controlManualWithJoystick();
			// System.err.println(motorController.getControlMode());
		}
		else if (!isFinished) {
			if (armControlMode == ArmControlMode.JOYSTICK_PID){
				controlPidWithJoystick();
				// System.err.println(motor1.getControlMode());
			}
			else if (armControlMode == ArmControlMode.SMART_MOTION){
				controlSMWithJoystick();
			}
			/* if (armControlMode == ArmControlMode.MOTION_PROFILE) {
				//isFinished = mpController.controlLoopUpdate(getPositionInches());

			}*/
			/*if (armControlMode == ArmControlMode.MOTION_MAGIC){
				controlMMWithJoystick();
				// System.err.println(motor1.getControlMode());
			}*/
			/*else if (armControlMode == ArmControlMode.MP_PATH_VELOCITY) {
				isFinished = mpPathVelocityController.controlLoopUpdate(getGyroAngleDeg());
			}
			else if (armControlMode == ArmControlMode.ADAPTIVE_PURSUIT) {
				updatePose();
				isFinished = adaptivePursuitController.controlLoopUpdate(currentPose);
			}*/
		}
	}

	/* public void resetZeroPosition(double position) {
		mpController.resetZeroPosition(position);
	} */

	public void resetEncoder(){
		motorEncoder.setPosition(0);
		//motor1.setEncPosition(0);
		//targetPositionInchesMM = 0;
		//targetPositionInchesPID = 0;
	}

	private synchronized void setArmControlMode(ArmControlMode controlMode) {
		this.armControlMode = controlMode;
	}

	private synchronized ArmControlMode getArmControlMode() {
		return this.armControlMode;
	}

	@Override
	public void setPeriodMs(long periodMs) {
		// mpController = new MPTalonPIDController(periodMs, motorControllers);
		// mpController.setPID(mpPIDParams, MP_SLOT);
		// mpController.setPID(armPIDParams, PID_SLOT);
		// mpController.setPIDSlot(PID_SLOT);
		this.periodMs = periodMs;
	}

	public void setSpeedJoystick(double speed) {
		setSpeed(speed);
		setArmControlMode(ArmControlMode.JOYSTICK_MANUAL);
	}
	
	public void setSpeed(double speed) {
		// motor1.set(ControlMode.PercentOutput, speed);
		motorController.setReference(speed, ControlType.kVoltage);
		setArmControlMode(ArmControlMode.MANUAL);
	}

	public double calcGravityCompensationAtCurrentPosition() {
		double rot = motorEncoder.getPosition();
		// double degreesFromDown = (rot+920)*(360.0/(4096*3));
		// double compensation = maxGravityComp * Math.sin(degreesFromDown*Math.PI/180);
		// System.err.println("comp(" + degreesFromDown + "^) = " + compensation);
		// return compensation;
		return 0.0;
	}

	private void ControlWithJoystickhold(){
		double holdPosition = motorEncoder.getPosition();
		updatePositionPID(holdPosition);
	}

	private void controlManualWithJoystick() {
		double joyStickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick(joyStickSpeed);
	}
	
	private void controlPidWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
		updatePositionPID(targetPositionInchesPID);
	}
	
	public void setPositionPID(double targetPositionInches) {
		// motorController.setReference(targetPositionInches, ControlType.kPosition);
		// mpController.setPIDSlot(PID_SLOT);	//TODO: verify that motor's selectProfileSlot() should be called AFTER its control mode is set
		updatePositionPID(targetPositionInches);
		setArmControlMode(ArmControlMode.JOYSTICK_PID);
		setFinished(false);
	}

	public void updatePositionPID(double targetPositionInches) {
		targetPositionInchesPID = limitPosition(targetPositionInches);
		if (limitPosition(motorEncoder.getPosition()) == MIN_POSITION_INCHES){
			resetEncoder();
		}
		double startPositionInches = motorEncoder.getPosition();
		// mpController.setTarget(targetPositionInchesPID, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN);
		motorController.setReference(targetPositionInches, ControlType.kPosition);
		// motor1.setClosedLoopRampRate(RampRate);
		motorController.setFF(targetPositionInchesPID > startPositionInches ? kFF_Up : kFF_Down);
		

		// motor1.configClosedloopRamp(0);
		// motor1.configPeakCurrentLimit(5);
		// motor1.configContinuousCurrentLimit(2);
		// motor1.config_kP(0, P_Value, TalonSRXEncoder.TIMEOUT_MS);
		// motor1.config_kI(0, I_Value, TalonSRXEncoder.TIMEOUT_MS);
		// motor1.config_kD(0, D_Value, TalonSRXEncoder.TIMEOUT_MS);
		// motor1.config_kF(0, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN, TalonSRXEncoder.TIMEOUT_MS);
		// System.err.println(motor1.getControlMode());
		// System.err.print(motor1.getClosedLoopError());
	}

	private void controlSMWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesSM = targetPositionInchesSM + deltaPosition;
		updatePositionSM(targetPositionInchesSM);
	}
	
	public void setPositionSM(double targetPositionInches) {
		// motorController.setReference(targetPositionInches, ControlType.kSmartMotion);
		updatePositionSM(targetPositionInches);
		setArmControlMode(ArmControlMode.SMART_MOTION);
		setFinished(false);
	}

	public void updatePositionSM(double targetPositionInches) {
		targetPositionInchesSM = limitPosition(targetPositionInches);
		if (limitPosition(motorEncoder.getPosition()) == MIN_POSITION_INCHES){
			resetEncoder();
		}

		double startPositionInches = motorEncoder.getPosition();
		motorController.setReference(targetPositionInches, ControlType.kSmartMotion);
		motorController.setFF(targetPositionInchesPID > startPositionInches ? kFF_Up : kFF_Down);
	}

	private double limitPosition(double targetPosition) {
		if (targetPosition < MIN_POSITION_INCHES) {
			return MIN_POSITION_INCHES;
		}
		if (targetPosition > MAX_POSITION_INCHES) {
			return MAX_POSITION_INCHES;
		}

		return targetPosition;
	}

	public double getPositionInches() {
		return motorEncoder.getPosition();
	}

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

	private int lastDPadAngle = -1;
	public void dPadButtons(){
		int dPadAngle = Robot.oi.getOperatorController().getDpadAngle();
		if (placeMode == PlaceMode.HATCH){
			if (dPadAngle == DPAD_UP && lastDPadAngle == -1){
				new HatchHigh().start();
			}
			if (dPadAngle == DPAD_RIGHT && lastDPadAngle == -1){
				new HatchMid().start();
			}
			if (dPadAngle == DPAD_DOWN && lastDPadAngle == -1){
				new HatchLow().start();
			}
		}
		if (placeMode == PlaceMode.CARGO) {
			if (dPadAngle == DPAD_UP && lastDPadAngle == -1){
				new CargoHigh().start();
			}
			if (dPadAngle == DPAD_RIGHT && lastDPadAngle == -1){
				new CargoMid().start();
			}
			if (dPadAngle == DPAD_DOWN && lastDPadAngle == -1){
				new CargoLow().start();
			}
		}

		if (dPadAngle == DPAD_LEFT && lastDPadAngle == -1){
			new StowArm().start();
		}
		SmartDashboard.putNumber("DPad Angle", dPadAngle);
		lastDPadAngle = dPadAngle;
	}

	/* public void setPositionMM(double targetPositionInches){
		//TODO// motor1.set(ControlMode.MotionMagic, targetPositionInches);
		//System.err.println(motor1.getControlMode());
		//TODO// motor1.selectProfileSlot(MM_SLOT, 0);
		setArmControlMode(ArmControlMode.MOTION_MAGIC);
		updatePositionMM(targetPositionInches);
		setFinished(false);
	} */

	/* public void setPositionMP(double targetPositionInches) {
		double startPositionInches = motorEncoder.getPosition();
		// mpController.setMPTarget(startPositionInches, limitPosition(targetPositionInches), MP_MAX_VELOCITY_INCHES_PER_SEC, MP_T1, MP_T2);
		setFinished(false);
		firstMpPoint = true;
		setArmControlMode(ArmControlMode.MOTION_PROFILE);
 	} */

	/* public void updatePositionMM(double targetPositionInches){
		targetPositionInchesMM = limitPosition(targetPositionInches);
		//double startPositionInches = motor1.getPositionWorld();
		double compensation = calcGravityCompensationAtCurrentPosition();
		//System.err.println("compensation = " + compensation);
		// motor1.set(ControlMode.MotionMagic, targetPositionInches);
		//TODO// motor1.set(ControlMode.MotionMagic, targetPositionInches, DemandType.ArbitraryFeedForward, compensation);
		//System.err.println(motor1.getControlMode());
		//TODO// motor1.configMotionCruiseVelocity(CV_value, TalonSRXEncoder.TIMEOUT_MS);
		//TODO// motor1.configMotionAcceleration(A_value, TalonSRXEncoder.TIMEOUT_MS);


	}*/

	/* private void controlMMWithJoystick() {
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
		double deltaPosition = joystickPosition * joystickInchesPerMs;
		targetPositionInchesMM = targetPositionInchesMM + deltaPosition;
		updatePositionMM(targetPositionInchesMM);
		//Robot.wrist.targetPositionInchesPID = targetPositionInchesPID - (deltaPosition/3);
		//Robot.wrist.updatePositionPID(Robot.wrist.targetPositionInchesPID);
	} */

	public void updateStatus(Robot.OperationMode operationMode) {
		//System.err.println("the encoder is right after this");
			try {

				SmartDashboard.putNumber("Arm Ticks", motorEncoder.getPosition());
				//SmartDashboard.putNumber("Arm Motor 1 Amps", motor1.getOutputCurrent());
				//SmartDashboard.putNumber("Arm Motor 2 Amps", motor2.getOutputCurrent());
				//SmartDashboard.putNumber("sensor vel", motor1.getSelectedSensorVelocity());
				//SmartDashboard.putNumber("Arm Average Amps", getAverageMotorCurrent());
				//SmartDashboard.putNumber("Arm Error", motor1.getClosedLoopError());
				SmartDashboard.putNumber("Arm Amps", getAverageMotorCurrent());
				SmartDashboard.putNumber("Arm Target SM", targetPositionInchesSM);
				SmartDashboard.putString("Arm Control Mode", armControlMode.toString());
				//SmartDashboard.putNumber("arm output", motor1.getMotorOutputPercent());
				//SmartDashboard.putNumber("Elevator Motor 1 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_1_CAN_ID));
				//SmartDashboard.putNumber("Elevator Motor 2 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_2_CAN_ID));
				//SmartDashboard.putNumber("Elevator Motor 3 Amps PDP", Robot.pdp.getCurrent(RobotMap.ELEVATOR_MOTOR_3_CAN_ID));
				//SmartDashboard.putNumber("Arm Target PID Position", targetPositionInchesPID);
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

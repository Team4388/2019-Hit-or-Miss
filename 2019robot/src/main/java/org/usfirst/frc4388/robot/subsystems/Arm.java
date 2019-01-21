package org.usfirst.frc4388.robot.subsystems;

import java.util.ArrayList;

import org.usfirst.frc4388.controller.XboxController;
import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.robot.commands.*;
import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.utility.CANTalonEncoder;
import org.usfirst.frc4388.utility.ControlLoopable;
import org.usfirst.frc4388.utility.PIDParams;
import org.usfirst.frc4388.utility.SoftwarePIDPositionController;

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

public class Arm extends Subsystem implements ControlLoopable
{
	//PID encoder and motor
	private CANTalonEncoder ArmMotor;
	//private WPI_TalonSRX ArmLeft;

	//PID controller Max Scale
	private SoftwarePIDPositionController pidPositionControllerMaxScale;
	//private PIDParams PositionPIDParamsMaxScale = new PIDParams(2.0, 0.0, 0.0);
	private PIDParams PositionPMaxScale;
	
	//PID controller Max Scale
	private SoftwarePIDPositionController pidPositionControllerLowScale;
	//private PIDParams PositionPIDParamsLowScale = new PIDParams(2.0, 0.0, 0.0);
	private PIDParams PositionPLowScale;
	
	//PID controller Max Scale
	private SoftwarePIDPositionController pidPositionControllerSwitch;
	//private PIDParams PositionPIDParamsSwitch = new PIDParams(2.0, 0.0, 0.0);
	private PIDParams PositionPSwitch;
	
	//PID controller Max Scale
	private SoftwarePIDPositionController pidPositionControllerLowest;
	//private PIDParams PositionPIDParamsLowest = new PIDParams(2.0, 0.0, 0.0);
	private PIDParams PositionPLowest;

	//PID target
	private double targetPPosition;
	
	//Encoder ticks to inches for encoders
	public static final double ENCODER_TICKS_TO_INCHES = Constants.kArmEncoderTicksPerInch;
	
	//control mode for joystick control
	private DriveControlMode controlMode = DriveControlMode.JOYSTICK;
	
	private double periodMs;
	
	//Non Linear Joystick
	public static final double STICK_DEADBAND = 0.02;
	public static final double MOVE_NON_LINEARITY = 1.0;
	private int moveNonLinear = 0;
	private double moveScale = 1.0;
	private double moveTrim = 0.0;
	
	private boolean isFinished;
	private DifferentialDrive m_drive;
	
	//private LimitSwitchSource limitSwitch = new DigitalInput(1);
	LimitSwitchSource limitSwitchSource;
	
	public boolean pressed;
	SensorCollection isPressed;
	
	public boolean ArmControlMode = false;
	// Motor controllers
	//private ArrayList<CANTalonEncoder> motorControllers = new ArrayList<CANTalonEncoder>();	
    
    public Arm()
    {    	
    	try
    	{
			//PID Arm encoder and talon
			ArmMotor = new CANTalonEncoder(RobotMap.ARM_MOTOR1_ID, ENCODER_TICKS_TO_INCHES, FeedbackDevice.QuadEncoder);
		
			//ArmLeft = new WPI_TalonSRX(RobotMap.ARM_MOTOR2_ID);
			
    		//ArmMotor.setInverted(false);

			//Setting left Arm motor as follower
    		//ArmLeft.set(ControlMode.Follower, ArmMotor.getDeviceID());
    		//ArmLeft.setInverted(false);
    		//ArmLeft.setNeutralMode(NeutralMode.Brake);
    		ArmMotor.setNeutralMode(NeutralMode.Brake);
    		ArmMotor.setSensorPhase(true);
    		//Limit Switch Left
    		//ArmLeft.overrideLimitSwitchesEnable(true);
    		//ArmLeft.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
    		//ArmLeft.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
    		
    		//Limit Switch Right
    		//ArmMotor.overrideLimitSwitchesEnable(true);
			//ArmMotor.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
    		//ArmMotor.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
    		
    		
    		//Change This boi
    		
    	//	ArmMotor.configForwardSoftLimitThreshold(10000, 0); //right here
    		//ArmMotor.configReverseSoftLimitThreshold(5, 0);
    		//ArmMotor.configForwardSoftLimitEnable(true, 0);
    		//ArmMotor.configReverseSoftLimitEnable(true, 0);
    		
    		//sos
    		//ArmMotor.enableLimitSwitch(true, true);

    		
    		
    		
    		
    	}
    	catch(Exception e)
    	{
    		System.err.println("You thought the code would work, but it was me, error. An error occurred in the Arm Construtor");
    	}
    }
    
    private double nonlinearStickCalcPositive(double move, double moveNonLinearity) 
    {
		return Math.sin(Math.PI / 2.0 * moveNonLinearity * move) / Math.sin(Math.PI / 2.0 * moveNonLinearity);
	}
    
    private double nonlinearStickCalcNegative(double move, double moveNonLinearity) 
    {
		return Math.asin(moveNonLinearity * move) / Math.asin(moveNonLinearity);
	}
    
    private boolean inDeadZone(double input) 
    {
		boolean inDeadZone;
		if (Math.abs(input) < STICK_DEADBAND)
		{
			inDeadZone = true;
		} 
		else 
		{
			inDeadZone = false;
		}
		
		return inDeadZone;
	}
    
    private double limitValue(double value) 
    {
		if (value > 1.0)
		{
			value = 1.0;
		}
		else if (value < -1.0)
		{
			value = -1.0;
		}
		return value;
	}
    
    public double adjustJoystickSensitivity(double scale, double trim, double move, int nonLinearFactor, double wheelNonLinearity) 
    {
		if (inDeadZone(move))
		{
			return 0;
		}

		move += trim;
		move *= scale;
		move = limitValue(move);

		int iterations = Math.abs(nonLinearFactor);
		for (int i = 0; i < iterations; i++)
		{
			if (nonLinearFactor > 0)
			{
				move = nonlinearStickCalcPositive(move, wheelNonLinearity);
			} 
			else
			{
				move = nonlinearStickCalcNegative(move, wheelNonLinearity);
			}
		}
		return move;
	}
    
    public void setArmMode()
    {
    	setControlMode(DriveControlMode.JOYSTICK);
    }
    
    public void resetArmEncoder()
    {
    	ArmMotor.setSelectedSensorPosition(0, 0, 0);
    }
    
    public void moveArmXbox()
    {
    	double moveArmInput;
    	double ArmSafeZone = 15; 
    	
    	double ArmPos = getEncoderArmPosition();
    	
    	moveArmInput = Robot.oi.getOperatorController().getLeftYAxis();
    	
    	//double moveArmSensitivity = adjustJoystickSensitivity(moveScale, moveTrim, moveArmInput, moveNonLinear, MOVE_NON_LINEARITY);
    	
    	boolean holdButtonPressed = Robot.oi.getOperatorJoystick().getRawButton(XboxController.A_BUTTON);
    	boolean ArmTuningPressed = Robot.oi.getOperatorJoystick().getRawButton(XboxController.Y_BUTTON);
    	
    	if(ArmTuningPressed == true)
      	{     		
     		ArmMotor.set(moveArmInput * 0.5);
      	}
     	else if(ArmTuningPressed == false)
     	{
     		ArmMotor.set(moveArmInput);
     	}
     		
     		/*
     		if(ArmPos <= ArmSafeZone && ArmPos >= 0)
     		{
     			ArmMotor.set(moveArmInput);
     		}
     		else if(ArmPos > ArmSafeZone)
     		{
     			ArmMotor.set(moveArmInput * 0.65);
     			
     			
     			if(holdButtonPressed == true)
             	{
             		ArmMotor.set(-0.43 * (0.2));
             	}
             	else if(holdButtonPressed == false)
             	{
             		ArmMotor.set(moveArmInput * 0.75);
             	}
             	
     		}
     		
     		else if(ArmPos < 0)
     		{
             	ArmMotor.set(moveArmInput * 0.75);
     		}
     		*/
     	}

     	
//     	System.out.println(ArmPos);		//-6.9 to 1.9   total: 8.8 range
	
    
	//PID encoder position
	public double getEncoderArmPosition()
	{
		return ArmMotor.getPositionWorld();
	}
	
	public double getArmHeightInchesAboveFloor()
	{
		return ArmMotor.getPositionWorld();
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
	
	public void setArmPIDLowest(double ArmPosition, double maxError, double minError)
	{
		double ArmTargetPos = 0;
		this.targetPPosition = ArmTargetPos;
		pidPositionControllerMaxScale.setPIDPositionTarget(ArmTargetPos, maxError, minError);
		Robot.Arm.setControlMode(DriveControlMode.MOVE_POSITION_LOWEST);
	}
	*/
	public void rawSetOutput(double output){
		ArmMotor.set(/*ControlMode.PercentOutput,*/ output);
	}
	
	public void holdInPos()
	{
		ArmMotor.set(-0.43 * 0.2);
	}
	
	public void stopMotors()
	{
		ArmMotor.set(0);
	}
	
	public void isSwitchPressed()
	{
		pressed = false;
		isPressed = ArmMotor.getSensorCollection();
		
		if(isPressed.isFwdLimitSwitchClosed() == true)
		{
			if (controlMode == DriveControlMode.JOYSTICK) {
				Robot.Arm.setControlMode(DriveControlMode.STOP_MOTORS);	
			}
			pressed = true;
		}
		else
		{
			if 	(controlMode == DriveControlMode.STOP_MOTORS){
				{
				Robot.Arm.setControlMode(DriveControlMode.JOYSTICK);
				}
			
			pressed = false;
			}
		}

	}
		
		//pressed = (isPressed.isFwdLimitSwitchClosed() == true) ? true : false;
	
	
	


	@Override
	public void controlLoopUpdate() 
	{
		if (controlMode == DriveControlMode.JOYSTICK || controlMode == DriveControlMode.CLIMB) 
		{
			moveArmXbox();
		}
		else if (!isFinished)
		{
			//PID control mode
			if(controlMode == DriveControlMode.MOVE_POSITION_MAX_SCALE)
			{
				isFinished = pidPositionControllerMaxScale.controlLoopUpdate(getEncoderArmPosition());
			}
			else if(controlMode == DriveControlMode.MOVE_POSITION_LOW_SCALE)
			{
				isFinished = pidPositionControllerLowScale.controlLoopUpdate(getEncoderArmPosition());
			}
			else if(controlMode == DriveControlMode.MOVE_POSITION_SWITCH)
			{
				isFinished = pidPositionControllerSwitch.controlLoopUpdate(getEncoderArmPosition());
			}
			else if(controlMode == DriveControlMode.MOVE_POSITION_LOWEST)
			{
				isFinished = pidPositionControllerLowest.controlLoopUpdate(getEncoderArmPosition());
			}
			/*
			else if(controlMode == DriveControlMode.RAW)
			{
				isFinished = pidPositionControllerLowest.controlLoopUpdate(getEncoderArmPosition());
			}
			*/
		}
	}

	@Override
    public void initDefaultCommand()
    {
    }

    @Override
    public void periodic() 
    {
    //	isSwitchPressed();
    }
    
	@Override
	public void setPeriodMs(long periodMs)
	{
		//PID controller
		pidPositionControllerMaxScale = new SoftwarePIDPositionController(PositionPMaxScale, ArmMotor);
		pidPositionControllerLowScale = new SoftwarePIDPositionController(PositionPLowScale, ArmMotor);
		pidPositionControllerSwitch = new SoftwarePIDPositionController(PositionPSwitch, ArmMotor);
		pidPositionControllerLowest = new SoftwarePIDPositionController(PositionPLowest, ArmMotor);
		
		this.periodMs = periodMs;
	}
	
	public synchronized boolean isFinished() 
	{
		return isFinished;
	}
	
	public double getPeriodMs()
	{
		return periodMs;
	}
	
	public void updateStatus(Robot.OperationMode operationMode) 
	{
		if (operationMode == Robot.OperationMode.TEST) 
		{
			try 
			{
				SmartDashboard.putNumber("Arm Pos Ticks", ArmMotor.getSelectedSensorPosition(0));
				SmartDashboard.putNumber("Arm Pos Inches", getArmHeightInchesAboveFloor());
				//SmartDashboard.putData(pressed);
			}
			catch (Exception e) 
			{
				System.err.println("Drivetrain update status error");
			}
		}
	}

	
}


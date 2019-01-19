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

public class Elevator extends Subsystem implements ControlLoopable
{
	//PID encoder and motor
	private CANTalonEncoder elevatorMain;
	
	
	//Encoder ticks to inches for encoders
	public static final double ENCODER_TICKS_TO_INCHES = Constants.kElevatorEncoderTicksPerInch;
	
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
	
	public boolean elevatorControlMode = false;
	// Motor controllers
	//private ArrayList<CANTalonEncoder> motorControllers = new ArrayList<CANTalonEncoder>();	
    
    public Elevator()
    {    	
    	try
    	{
			//PID elevator encoder and talon
			elevatorMain = new CANTalonEncoder(RobotMap.ELEVATOR_MOTOR1_ID, ENCODER_TICKS_TO_INCHES, FeedbackDevice.QuadEncoder);
		
			
    		elevatorMain.setInverted(false);

			//Setting left elevator motor as follower
    		elevatorMain.setNeutralMode(NeutralMode.Brake);
    		elevatorMain.setSensorPhase(true);
    		//Limit Switch Left
    		//elevatorLeft.overrideLimitSwitchesEnable(true);
    		elevatorMain.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
    		elevatorMain.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
    		
    		//Limit Switch Right
    		//elevatorMain.overrideLimitSwitchesEnable(true);
			//elevatorMain.configForwardLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
    		//elevatorMain.configReverseLimitSwitchSource(limitSwitchSource, LimitSwitchNormal.NormallyOpen, 0);
    		
    		
    		//Change This boi
    		
    	//	elevatorMain.configForwardSoftLimitThreshold(10000, 0); //right here
    		//elevatorMain.configReverseSoftLimitThreshold(5, 0);
    		//elevatorMain.configForwardSoftLimitEnable(true, 0);
    		//elevatorMain.configReverseSoftLimitEnable(true, 0);
    		
    		//sos
    		//elevatorMain.enableLimitSwitch(true, true);

    		
    		
    		
    		
    	}
    	catch(Exception e)
    	{
    		System.err.println("You thought the code would work, but it was me, error. An error occurred in the Elevator Construtor");
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
    
    public void setElevatorMode()
    {
    	setControlMode(DriveControlMode.JOYSTICK);
    }
    
    public void resetElevatorEncoder()
    {
    	elevatorMain.setSelectedSensorPosition(0, 0, 0);
    }
    
    public void moveElevatorXbox()
    {
    	double moveElevatorInput;
    	double elevatorSafeZone = 15; 
    	
    	double elevatorPos = getEncoderElevatorPosition();
    	
    	moveElevatorInput = Robot.oi.getOperatorController().getLeftYAxis();
    	
    	//double moveElevatorSensitivity = adjustJoystickSensitivity(moveScale, moveTrim, moveElevatorInput, moveNonLinear, MOVE_NON_LINEARITY);
    	
    	boolean holdButtonPressed = Robot.oi.getOperatorJoystick().getRawButton(XboxController.A_BUTTON);
    	boolean elevatorTuningPressed = Robot.oi.getOperatorJoystick().getRawButton(XboxController.Y_BUTTON);
    	
    	if(elevatorTuningPressed == true)
      	{     		
     		elevatorMain.set(moveElevatorInput * 0.5);
      	}
     	else if(elevatorTuningPressed == false)
     	{
     		elevatorMain.set(moveElevatorInput);
     	}
     		
     		/*
     		if(elevatorPos <= elevatorSafeZone && elevatorPos >= 0)
     		{
     			elevatorMain.set(moveElevatorInput);
     		}
     		else if(elevatorPos > elevatorSafeZone)
     		{
     			elevatorMain.set(moveElevatorInput * 0.65);
     			
     			
     			if(holdButtonPressed == true)
             	{
             		elevatorMain.set(-0.43 * (0.2));
             	}
             	else if(holdButtonPressed == false)
             	{
             		elevatorMain.set(moveElevatorInput * 0.75);
             	}
             	
     		}
     		
     		else if(elevatorPos < 0)
     		{
             	elevatorMain.set(moveElevatorInput * 0.75);
     		}
     		*/
     	}

     	
//     	System.out.println(elevatorPos);		//-6.9 to 1.9   total: 8.8 range
	
    
	//PID encoder position
	public double getEncoderElevatorPosition()
	{
		return elevatorMain.getPositionWorld();
	}
	
	public double getElevatorHeightInchesAboveFloor()
	{
		return elevatorMain.getPositionWorld();
	}

	public synchronized void setControlMode(DriveControlMode controlMode) 
	{
 		this.controlMode = controlMode;
 		
 		isFinished = false;
	}
	/*
	public void setElevatorPIDMaxScale(double ElevatorPosition, double maxError, double minError)
	{
		double elevatorTargetPos = 0;
		this.targetPPosition = elevatorTargetPos;
		pidPositionControllerMaxScale.setPIDPositionTarget(elevatorTargetPos, maxError, minError);      ///////TARGET POSITION WHERE??
		Robot.elevator.setControlMode(DriveControlMode.MOVE_POSITION_MAX_SCALE);
	}
	
	public void setElevatorPIDLowScale(double ElevatorPosition, double maxError, double minError)
	{
		double elevatorTargetPos = 0;
		this.targetPPosition = elevatorTargetPos;
		pidPositionControllerMaxScale.setPIDPositionTarget(elevatorTargetPos, maxError, minError);
		Robot.elevator.setControlMode(DriveControlMode.MOVE_POSITION_LOW_SCALE);
	}
	
	public void setElevatorPIDSwitch(double ElevatorPosition, double maxError, double minError)
	{
		double elevatorTargetPos = 0;
		this.targetPPosition = elevatorTargetPos;
		pidPositionControllerMaxScale.setPIDPositionTarget(elevatorTargetPos, maxError, minError);
		Robot.elevator.setControlMode(DriveControlMode.MOVE_POSITION_SWITCH);
	}
	
	public void setElevatorPIDLowest(double ElevatorPosition, double maxError, double minError)
	{
		double elevatorTargetPos = 0;
		this.targetPPosition = elevatorTargetPos;
		pidPositionControllerMaxScale.setPIDPositionTarget(elevatorTargetPos, maxError, minError);
		Robot.elevator.setControlMode(DriveControlMode.MOVE_POSITION_LOWEST);
	}
	*/
	public void rawSetOutput(double output){
		elevatorMain.set(/*ControlMode.PercentOutput,*/ output);
	}
	
	public void holdInPos()
	{
		elevatorMain.set(-0.43 * 0.2);
	}
	
	public void stopMotors()
	{
		elevatorMain.set(0);
	}
	
	public void isSwitchPressed()
	{
		pressed = false;
		isPressed = elevatorMain.getSensorCollection();
		
		if(isPressed.isFwdLimitSwitchClosed() == true)
		{
			if (controlMode == DriveControlMode.JOYSTICK) {
				Robot.elevator.setControlMode(DriveControlMode.STOP_MOTORS);	
			}
			pressed = true;
		}
		else
		{
			if 	(controlMode == DriveControlMode.STOP_MOTORS){
				{
				Robot.elevator.setControlMode(DriveControlMode.JOYSTICK);
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
			moveElevatorXbox();
		}
		else if (!isFinished)
		{
			//PID control mode
			/*
			else if(controlMode == DriveControlMode.RAW)
			{
				isFinished = pidPositionControllerLowest.controlLoopUpdate(getEncoderElevatorPosition());
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
				SmartDashboard.putNumber("Elevator Pos Ticks", elevatorMain.getSelectedSensorPosition(0));
				SmartDashboard.putNumber("Elevator Pos Inches", getElevatorHeightInchesAboveFloor());
				//SmartDashboard.putData(pressed);
			}
			catch (Exception e) 
			{
				System.err.println("Drivetrain update status error");
			}
		}
	}

	
}


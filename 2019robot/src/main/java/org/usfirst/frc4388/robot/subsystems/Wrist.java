package org.usfirst.frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import java.util.ArrayList;

import org.usfirst.frc4388.controller.XboxController;
import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.utility.MPTalonPIDController;
import org.usfirst.frc4388.robot.commands.*;
import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.utility.CANTalonEncoder;
import org.usfirst.frc4388.utility.ControlLoopable;
import org.usfirst.frc4388.utility.PIDParams;
import org.usfirst.frc4388.utility.SoftwarePIDPositionController;
import org.usfirst.frc4388.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Add your docs here.
 */
public class Wrist extends Subsystem
{
  //Control Mode Array
  public static enum WristControlMode {PID, JOYSTICK_MANUAL, GRAB_BALL};

  //Motor Controllers
  private ArrayList<CANTalonEncoder> motorControllers = new ArrayList<CANTalonEncoder>();

	private CANTalonEncoder wristRight;

  //Encoder ticks to inches for encoders
  public static final double ENCODER_TICKS_TO_INCHES = Constants.kArmEncoderTicksPerDegree;
  public static final double ENCODER_TICKS_TO_DEGREES = Constants.kWristEncoderTicksPerDegree * Math.PI;
  
  // PID controller and params
	private MPTalonPIDController mpController;

	public static int PID_SLOT = 0;
	public static int MP_SLOT = 1;

	private PIDParams mpPIDParams = new PIDParams(0.2, 0.0, 0.0, 0.0, 0.005, 0.0);  
	private PIDParams pidPIDParamsHiGear = new PIDParams(0.075, 0.0, 0.0, 0.0, 0.0, 0.0);  
	private PIDParams pidPIDParamsLoGear = new PIDParams(0.45, 0.0, 0.0, 0.0, 0.0, 0.0);  
	public static final double KF_UP = 0.005;
	public static final double KF_DOWN = 0.0;
  public static final double PID_ERROR_INCHES = 1.0;

  // Defined positions
	public static final double MIN_POSITION_INCHES = 0.0;
  public static final double MAX_POSITION_INCHES = 83.4;

  public static final double MIN_ANGLE_DEGREES = -3;   ////FIND ANGLE VALUES
  public static final double MAX_ANGLE_DEGREES = 3;

  public static final double JOYSTICK_INCHES_PER_MS_HI = 0.75;
  public static final double JOYSTICK_INCHES_PER_MS_LO = JOYSTICK_INCHES_PER_MS_HI/3.68 * 0.8;
  public static final double JOYSTICK_Degrees_PER_MS_LO = JOYSTICK_INCHES_PER_MS_HI/3.68 * 0.8;

  public double armAngleDegrees = Robot.arm.ARM_ANGLE_DEGREES;
  
  //Misc
  private WristControlMode wristControlMode = WristControlMode.JOYSTICK_MANUAL;
  private boolean isFinished;
  private double targetPositionInchesPID = 0;
  private double targetAngleDegreesPID = -(180 - armAngleDegrees);

  private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
  private double joystickDegreesPerMs = JOYSTICK_Degrees_PER_MS_LO;
  
  public Wrist()
  {
    try
    {
      //PID wrist encoder and talon
			wristRight = new CANTalonEncoder(RobotMap.WRIST_LEFT_MOTOR_CAN_ID, ENCODER_TICKS_TO_DEGREES, FeedbackDevice.QuadEncoder);
    }
    catch(Exception e)
    {
      System.err.println("You thought the code would work, but it was me, error. An error occurred in the Wrist Construtor");
    }
  }

  //Method for setting the control mode for the wrist
  private synchronized void setWristControlMode(WristControlMode controlMode) 
  {
		this.wristControlMode = controlMode;
  }
  
  //Getting the control mode for the wrist
  private synchronized WristControlMode getWristControlMode() 
  {
		return this.wristControlMode;
  }

  //Setting the speed for the motor on the wrist along with setting the control mode to manual
  public void setSpeed(double speed) 
  {
		wristRight.set(ControlMode.PercentOutput, speed);
		setWristControlMode(WristControlMode.JOYSTICK_MANUAL);
	}

  //Setting the target position for the PID loop and set the control mode to PID
  public void setPositionPID(double targetAngleInches) 
  {
    mpController.setPIDSlot(PID_SLOT);
		updatePositionPID(targetAngleInches);
		setWristControlMode(WristControlMode.PID);	
		setFinished(false);
  }
  
  //Setting range for target position
  private double limitPosition(double targetAngle) 
  {
		if (targetAngle < MIN_ANGLE_DEGREES) {
			return MIN_ANGLE_DEGREES;
		}
		else if (targetAngle > MAX_ANGLE_DEGREES) {
			return MAX_ANGLE_DEGREES;
		}
		
		return targetAngle;
	}
  
  //Method for updating the PID target position
  public void updatePositionPID(double targetAngleDegrees) 
  {
 		targetAngleDegreesPID = limitPosition(targetAngleDegrees);
		double startAngleDegrees = wristRight.getPositionWorld();
		mpController.setTarget(targetPositionInchesPID, targetAngleDegreesPID > startAngleDegrees ? KF_UP : KF_DOWN); 
  }

  //Getting the current encoder position
  public double getPositionDegrees() 
  {
		return wristRight.getPositionWorld();
  }
  
  //Setting the speed for the motors in manual mode
  public void setSpeedJoystick(double speed) 
  {
		wristRight.set(ControlMode.PercentOutput, speed);
		setWristControlMode(wristControlMode.JOYSTICK_MANUAL);
  }

  //@Override
  public void onLoop(double timestamp) 
  {
		synchronized (Wrist.this) {
			switch(getWristControlMode() ) {
				case PID: 
					controlPID();
					break;
				case JOYSTICK_MANUAL:
					controlManualWithJoystick();
					break;
				default:
					break;
			}
		}
  }
  
  private void controlPID() 
  {
		double joystickAngle = -Robot.oi.getOperatorController().getLeftYAxis();
    double deltaAngle = joystickAngle * joystickDegreesPerMs;

		targetAngleDegreesPID = targetAngleDegreesPID + deltaAngle;
    updatePositionPID(targetAngleDegreesPID);
	}
  
  //Method for controlling the motor with the joystick
  private void controlManualWithJoystick() 
  {
    double joystickSpeed;
    
    joystickSpeed = -Robot.oi.getOperatorController().getLeftYAxis();
		setSpeedJoystick(joystickSpeed);
	}

  public synchronized boolean isFinished() 
  {
		return isFinished;
	}
  
  public synchronized void setFinished(boolean isFinished) 
  {
		this.isFinished = isFinished;
	}

  @Override
  public void initDefaultCommand() 
  {
  }

  public void updateStatus(Robot.OperationMode operationMode) 
	{
		if (operationMode == Robot.OperationMode.TEST) 
		{
			try 
			{
			}
			catch (Exception e)
			{
				System.err.println("Wrist update status error");
			}
		}
	}
}
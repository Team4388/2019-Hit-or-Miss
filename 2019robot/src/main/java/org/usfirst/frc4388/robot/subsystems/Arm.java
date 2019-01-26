
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
public class Arm extends Subsystem
{
  //Control Mode Array
  public static enum ArmControlMode {PID, JOYSTICK_MANUAL};

  //Motor Controllers
  private ArrayList<CANTalonEncoder> motorControllers = new ArrayList<CANTalonEncoder>();

	private CANTalonEncoder arm1;

  //Encoder ticks to inches for encoders
  public static final double ENCODER_TICKS_TO_INCHES = Constants.kArmEncoderTicksPerDegree;
  
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
  public static final double JOYSTICK_INCHES_PER_MS_HI = 0.75;
  public static final double JOYSTICK_INCHES_PER_MS_LO = JOYSTICK_INCHES_PER_MS_HI/3.68 * 0.8;
  
  //Misc
  private ArmControlMode armControlMode = ArmControlMode.JOYSTICK_MANUAL;
  private boolean isFinished;
  private double targetPositionInchesPID = 0;

  private double joystickInchesPerMs = JOYSTICK_INCHES_PER_MS_LO;
  
  public Arm()
  {
    try
    {
      //PID arm encoder and talon
			arm1 = new CANTalonEncoder(RobotMap.ARM_MOTOR1_ID, ENCODER_TICKS_TO_INCHES, FeedbackDevice.QuadEncoder);
    }
    catch(Exception e)
    {
      System.err.println("You thought the code would work, but it was me, error. An error occurred in the Arm Construtor");
    }
  }

  //Method for setting the control mode for the arm
  private synchronized void setArmControlMode(ArmControlMode controlMode) 
  {
		this.armControlMode = controlMode;
  }
  
  //Getting the control mode for the arm
  private synchronized ArmControlMode getArmControlMode() 
  {
		return this.armControlMode;
  }

  //Setting the speed for the motor on the arm along with setting the control mode to manual
  public void setSpeed(double speed) 
  {
		arm1.set(ControlMode.PercentOutput, speed);
		setArmControlMode(ArmControlMode.JOYSTICK_MANUAL);
	}

  //Setting the target position for the PID loop and set the control mode to PID
  public void setPositionPID(double targetPositionInches) 
  {
    mpController.setPIDSlot(PID_SLOT);
		updatePositionPID(targetPositionInches);
		setArmControlMode(ArmControlMode.PID);	
		setFinished(false);
  }
  
  //Setting range for target position
  private double limitPosition(double targetPosition) {
		if (targetPosition < MIN_POSITION_INCHES) {
			return MIN_POSITION_INCHES;
		}
		else if (targetPosition > MAX_POSITION_INCHES) {
			return MAX_POSITION_INCHES;
		}
		
		return targetPosition;
	}
  
  //Method for updating the PID target position
  public void updatePositionPID(double targetPositionInches) 
  {
 		targetPositionInchesPID = limitPosition(targetPositionInches);
		double startPositionInches = arm1.getPositionWorld();
		mpController.setTarget(targetPositionInchesPID, targetPositionInchesPID > startPositionInches ? KF_UP : KF_DOWN); 
  }

  //Getting the current encoder position
  public double getPositionInches() 
  {
		return arm1.getPositionWorld();
  }
  
  //Setting the speed for the motors in manual mode
  public void setSpeedJoystick(double speed) 
  {
		arm1.set(ControlMode.PercentOutput, speed);
		setArmControlMode(armControlMode.JOYSTICK_MANUAL);
  }

  //@Override
  public void onLoop(double timestamp) 
  {
		synchronized (Arm.this) {
			switch(getArmControlMode() ) {
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
		double joystickPosition = -Robot.oi.getOperatorController().getLeftYAxis();
    double deltaPosition = joystickPosition * joystickInchesPerMs;

		targetPositionInchesPID = targetPositionInchesPID + deltaPosition;
    updatePositionPID(targetPositionInchesPID);
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
				System.err.println("Arm update status error");
			}
		}
	}
}

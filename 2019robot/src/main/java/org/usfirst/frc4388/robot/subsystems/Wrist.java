package org.usfirst.frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class Wrist extends Subsystem
{
  //PID encoder and motor
	private CANTalonEncoder wristRight;
  private WPI_TalonSRX wristLeft;

  //Encoder ticks to inches for encoders
	public static final double ENCODER_TICKS_TO_INCHES = Constants.kArmEncoderTicksPerInch;
  
  public Wrist()
  {
    try
    {
      //PID wrist encoder and talon
			wristRight = new CANTalonEncoder(RobotMap.WRIST_LEFT_MOTOR_CAN_ID, ENCODER_TICKS_TO_INCHES, FeedbackDevice.QuadEncoder);
      wristLeft = new WPI_TalonSRX(RobotMap.WRITST_RIGHT_MOTOR_CAN_ID);
      
      
    }
    catch(Exception e)
    {
      System.err.println("You thought the code would work, but it was me, error. An error occurred in the Wrist Construtor");
    }
  }

  @Override
  public void initDefaultCommand() 
  {
  }
}

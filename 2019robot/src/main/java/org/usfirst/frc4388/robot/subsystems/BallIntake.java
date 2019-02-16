package org.usfirst.frc4388.robot.subsystems;

import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.robot.commands.*;
import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.utility.CANTalonEncoder;
import org.usfirst.frc4388.utility.ControlLoopable;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc4388.robot.OI;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;



/**
 *
 */
public class BallIntake extends Subsystem {

	private WPI_TalonSRX BallIntakeMain;
	public static enum BallIntakeControlMode { JOYSTICK, MP_STRAIGHT, HOLD, MANUAL};
	public static final double BALL_INTAKE_SPEED = 0.40;
	public static final double BALL_EXTAKE_SPEED = -1.0;
	public static final double CUBE_STOP_SPEED = 0;
	/////^^^^^^^^^ replace this line with the modes we need
	
	
	private boolean isFinished;
	//private CarriageControlMode controlMode = CarriageControlMode.JOYSTICK;
	
	
	
	
	public BallIntake() {
		try {
			BallIntakeMain = new WPI_TalonSRX(RobotMap.CLIMBER_CAN_ID);
			//\][carriageLeft.set(CurrentLimit, value);
			
    }
		catch (Exception e) {
			System.err.println("An error occurred in the Ball Intake constructor");
			
			
		}
    }
	
		public void setWheelSpeed(double speed) {
			BallIntakeMain.set(-speed);
    }


    @Override
    public void periodic() {
    	
    }
    	public void initDefaultCommand() {
    }
}

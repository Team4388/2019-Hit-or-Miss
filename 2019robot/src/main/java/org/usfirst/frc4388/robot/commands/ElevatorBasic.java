package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorBasic extends Command {
    private double m_targetHeightInchesAboveFloor;
    private double m_speed;
    private boolean m_goingUp;
    private double m_commandInitTimestamp;
	private double m_lastCommandExecuteTimestamp = 0.0;
	private double m_lastCommandExecutePeriod = 0.0;
	public static boolean isfinishedElevatorBasic;
 
    public ElevatorBasic(double targetHeightInchesAboveFloor) {
        requires(Robot.elevator);
        m_targetHeightInchesAboveFloor = targetHeightInchesAboveFloor;
   }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.elevator.setControlMode(DriveControlMode.RAW);
    	
    	double currentHeight = Robot.elevator.getElevatorHeightInchesAboveFloor();
    	// start out at half speed, as crude way to reduce slippage
    	m_goingUp = (m_targetHeightInchesAboveFloor > currentHeight);
System.out.println("initialize(): cur=" + currentHeight + " , target=" + m_targetHeightInchesAboveFloor + " , going " + (m_goingUp ? "UP" : "DOWN"));
    	if (m_goingUp) {
    		m_speed = Constants.kElevatorBasicPercentOutputUp;
    	}
    	else {
    		m_speed = Constants.kElevatorBasicPercentOutputDown;
    	}
		m_commandInitTimestamp = Timer.getFPGATimestamp();
		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		// Measure *actual* update period
		double currentTimestamp = Timer.getFPGATimestamp();
		if (m_lastCommandExecuteTimestamp > 0.001)	{	// ie, if this is NOT the first time
			m_lastCommandExecutePeriod = currentTimestamp - m_lastCommandExecuteTimestamp;
		}
		m_lastCommandExecuteTimestamp = currentTimestamp;
    	Robot.elevator.rawSetOutput(m_speed);
		//SmartDashboard.putNumber("DSB Period", m_lastCommandExecutePeriod);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean finished=false;
    	double currentHeight = Robot.elevator.getElevatorHeightInchesAboveFloor();
    	double remaining = (m_targetHeightInchesAboveFloor - currentHeight) * (m_goingUp ? 1.0 : -1.0);
System.out.println("cur=" + currentHeight + " , remaining=" + remaining + " , target=" + m_targetHeightInchesAboveFloor);
    	if (remaining < 0.0) {
    		finished = true;
    		
    	}
    	/*} else if (remaining < 0.1 * m_maxVelocityInchesPerSec / 2.0) {	// last 100 ms
    		velocity = m_maxVelocityInchesPerSec / 4.0;		// quarter speed
    	} else if (remaining < 0.3 * m_maxVelocityInchesPerSec) {		// last 300 ms
    		velocity = m_maxVelocityInchesPerSec / 2.0;		// half speed
    	}*/
    		
    		
    	if (!finished) {
    		SmartDashboard.putNumber("EB Dist", currentHeight);
    	} else {
    		SmartDashboard.putNumber("EB finDist", currentHeight);
    	}
		return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
		double currentTimestamp = Timer.getFPGATimestamp();
    	SmartDashboard.putNumber("EB Runtime", currentTimestamp - m_commandInitTimestamp);
    	
    	isfinishedElevatorBasic = isFinished();
    	
    	Robot.elevator.rawSetOutput(0.0);
    	
		Robot.elevator.setControlMode(DriveControlMode.JOYSTICK);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}

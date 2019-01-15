package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveStraightBasic extends Command {
    private double m_targetInches;
    private double m_maxVelocityInchesPerSec;
    private double m_speed;
    private boolean m_goingBackwards;
    private boolean m_useGyroLock;
    private boolean m_useAbsolute;
    private double m_desiredAbsoluteAngle;
    private double m_commandInitTimestamp;
	private double m_lastCommandExecuteTimestamp = 0.0;
	private double m_lastCommandExecutePeriod = 0.0;
 
    public DriveStraightBasic(double targetInches, double maxVelocityInchesPerSec, boolean useGyroLock, boolean useAbsolute, double desiredAbsoluteAngle) {
        requires(Robot.drive);
        m_targetInches = targetInches;
        m_maxVelocityInchesPerSec = maxVelocityInchesPerSec;
        m_useGyroLock = useGyroLock;
        m_useAbsolute = useAbsolute;
        m_desiredAbsoluteAngle = desiredAbsoluteAngle;
        m_goingBackwards = (m_targetInches < 0.0);
   }
    
    protected double velocityToMoveSpeed(double velocityInchesPerSec, boolean backwards) {
    	double sign = (backwards ? -1.0 : 1.0);
    	// Keep velocity above stiction limit (else bot will freeze and command will not complete)
    	double velocity = Math.max(Constants.kDriveStraightBasicMinSpeedInchesPerSec, velocityInchesPerSec);
        // Figure out move value based on percentage of measured max speed (i.e. that at full 1.0 joystick)
    	return sign * velocity / Constants.kDriveStraightBasicMaxSpeedInchesPerSec;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.resetGyro();
    	Robot.drive.resetEncoders();
    	Robot.drive.setControlMode(DriveControlMode.RAW);
    	// start out at half speed, as crude way to reduce slippage
        m_speed = velocityToMoveSpeed(m_maxVelocityInchesPerSec/2.0, m_goingBackwards);
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
    	double steer = 0.0;
    	if (m_useGyroLock) {
    		steer = - Robot.drive.getGyroAngleDeg() / 25.0;	//TODO: tune
    	}
    	Robot.drive.rawMoveSteer(m_speed, steer);
		//SmartDashboard.putNumber("DSB Period", m_lastCommandExecutePeriod);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	boolean finished=false;
    	double velocity = m_maxVelocityInchesPerSec;
    	double position = (Robot.drive.getLeftPositionWorld() + Robot.drive.getRightPositionWorld())/2;
    	double remaining = (m_targetInches - position) * (m_goingBackwards ? -1.0 : 1.0);
    	if (remaining < 0.0) {
    		finished = true;
    	} else if (remaining < 0.1 * m_maxVelocityInchesPerSec / 2.0) {	// last 100 ms
    		velocity = m_maxVelocityInchesPerSec / 4.0;		// quarter speed
    	} else if (remaining < 0.3 * m_maxVelocityInchesPerSec) {		// last 300 ms
    		velocity = m_maxVelocityInchesPerSec / 2.0;		// half speed
    	}
    	if (!finished) {
    		m_speed = velocityToMoveSpeed(velocity, m_goingBackwards);
    		SmartDashboard.putNumber("DSB Dist", position);
    	} else {
    		SmartDashboard.putNumber("DSB finDist", position);
    	}
		return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
		double currentTimestamp = Timer.getFPGATimestamp();
    	SmartDashboard.putNumber("DSB Runtime", currentTimestamp - m_commandInitTimestamp);
    	Robot.drive.rawMoveSteer(0.0, 0.0);
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}

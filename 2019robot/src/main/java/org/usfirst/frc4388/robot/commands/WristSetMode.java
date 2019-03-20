package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Wrist.WristControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WristSetMode extends Command {

	private WristControlMode controlMode;
	
    public WristSetMode(WristControlMode controlMode) {
    	this.controlMode = controlMode;
        requires(Robot.wrist);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (controlMode == WristControlMode.JOYSTICK_PID) {
    		Robot.wrist.setPositionPID(Robot.wrist.getPositionInches());
    	}
    	else if (controlMode == WristControlMode.JOYSTICK_MANUAL) {
    		Robot.wrist.setSpeedJoystick(0);
        }
        else if (controlMode == WristControlMode.MOTION_MAGIC){
            Robot.wrist.setPositionMM(Robot.wrist.getPositionInches());
        }

    	else {
    		Robot.wrist.setSpeed(0.0);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

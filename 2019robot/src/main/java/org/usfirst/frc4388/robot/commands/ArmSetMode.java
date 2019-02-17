package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Arm.ArmControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmSetMode extends Command {

	private ArmControlMode controlMode;
	
    public ArmSetMode(ArmControlMode controlMode) {
    	this.controlMode = controlMode;
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (controlMode == ArmControlMode.JOYSTICK_PID) {
    		Robot.arm.setPositionPID(Robot.arm.getPositionInches());
    	}
    	else if (controlMode == ArmControlMode.JOYSTICK_MANUAL) {
    		Robot.arm.setSpeedJoystick(0);
    	}
    	else {
    		Robot.arm.setSpeed(0.0);
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

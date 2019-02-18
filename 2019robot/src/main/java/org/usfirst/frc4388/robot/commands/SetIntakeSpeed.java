package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import org.usfirst.frc4388.robot.subsystems.BallIntake;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetIntakeSpeed extends Command {

	private double WheelSpeed;
	
	// Constructor with speed
    public SetIntakeSpeed(double WheelSpeed) {
    	this.WheelSpeed = WheelSpeed;
         requires(Robot.ballIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.ballIntake.setWheelSpeed(WheelSpeed);
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
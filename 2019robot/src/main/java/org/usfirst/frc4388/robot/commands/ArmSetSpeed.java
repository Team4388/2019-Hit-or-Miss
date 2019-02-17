package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import org.usfirst.frc4388.robot.subsystems.*;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorSetSpeed extends Command {

	private double RaiseSpeed;
	
	// Constructor with speed
    public ElevatorSetSpeed(double RaiseSpeed) {
    	this.RaiseSpeed = RaiseSpeed;
       // requires(Robot.elevatorAuton);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Robot.elevatorAuton.setRaiseSpeed(RaiseSpeed);
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

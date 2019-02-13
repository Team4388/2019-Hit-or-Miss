package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.Constants;

import edu.wpi.first.wpilibj.command.Command;

public class InitiateClimber extends Command
{
	boolean climb;
	float speed;

	public InitiateClimber(boolean Climb, float speed) {
		requires(Robot.climber);
		this.climb = Climb;
		this.speed = speed;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
		if(climb){ //If climb and saftey button are pressed
			Robot.climber.setClimbSpeed(climb, speed);
		}
		else{
			Robot.climber.setClimbSpeed(false, 0);
		}
	}
	   // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

	@Override
	protected void interrupted() {

	}
}

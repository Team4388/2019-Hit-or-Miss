package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.Constants;

import edu.wpi.first.wpilibj.command.Command;

public class InitiateClimber extends Command
{
	static boolean CLIMB;
	double speed;

	public InitiateClimber(boolean climb, double speed) {
		requires(Robot.climber);
		this.CLIMB = climb;
		this.speed = speed;
	}

	@Override
	protected void initialize() {
		
	}

	@Override
	protected void execute() {
		if(CLIMB){ //If climb button is pressed
			Robot.climber.setClimbSpeed(CLIMB, speed);
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

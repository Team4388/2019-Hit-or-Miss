package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.Constants;

import edu.wpi.first.wpilibj.command.Command;

public class InitiateClimber extends Command
{
	
	public InitiateClimber() {
		requires(Robot.climber);
	}

	@Override
	protected void initialize() {
		Robot.climber.setClimbSpeed(kClimbLiftSpeed);
	}

	@Override
	protected void execute() {
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
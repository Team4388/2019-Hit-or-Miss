package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveSpeedShift extends Command
{
	public boolean speed;
	
	public DriveSpeedShift(boolean speed) {
		this.speed=speed;
		requires(Robot.pnumatics);
	}

	@Override
	protected void initialize() {
		Robot.pnumatics.setShiftState(speed);
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

	@Override
	protected void end() {
		
	}

	@Override
	protected void interrupted() {
			
	}
}
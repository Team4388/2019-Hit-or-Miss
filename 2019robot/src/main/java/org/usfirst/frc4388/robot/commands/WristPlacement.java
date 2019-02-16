package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class WristPlacement extends Command
{
	public boolean Forward;
	
	public WristPlacement(boolean Forward) {
		this.Forward=Forward;
		requires(Robot.pnumatics);
	}

	@Override
	protected void initialize() {
		Robot.pnumatics.setWrist(Forward);
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
package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ArmSetZero extends Command
{
	private double position;
	
	public ArmSetZero(double position) {
		this.position = position;
		requires(Robot.arm);
	}

	@Override
	protected void initialize() {
		Robot.arm.resetZeroPosition(position);
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
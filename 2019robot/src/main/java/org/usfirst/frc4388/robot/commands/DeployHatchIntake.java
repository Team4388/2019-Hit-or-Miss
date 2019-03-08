package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DeployHatchIntake extends Command
{
	public boolean deployed;
	
	public DeployHatchIntake(boolean deployed) {
		this.deployed = !deployed;
		requires(Robot.pnumatics);
	}

	@Override
	protected void initialize() {
		Robot.pnumatics.setHatchIntakeState(deployed);
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
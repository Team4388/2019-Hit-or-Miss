package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
/**
 * @deprecated use HatchAndBallSet insted for all setting of hatch and ball intakes
 */
public class HatchFlip extends Command
{
	public boolean PickingUp;
	
	public HatchFlip(boolean PickingUP) {
		this.PickingUp=PickingUP;
		requires(Robot.pnumatics);
	}

	@Override
	protected void initialize() {
		Robot.pnumatics.setHatchIntakeState(PickingUp);
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
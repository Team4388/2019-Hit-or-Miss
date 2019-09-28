package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.command.Command;

public class ArmAutoZero extends Command
{
	private double MIN_ELEVATOR_POSITION_CHANGE = 0.05;
	private double lastArmPosition;
	private int encoderCount;
	
	public ArmAutoZero(boolean interrutible) {
			requires(Robot.arm);
		setInterruptible(interrutible);
	}

	@Override
	protected void initialize() {
		lastArmPosition = Arm.MAX_POSITION_ROT;
		Robot.arm.setSpeed(Arm.AUTO_ZERO_SPEED);
		encoderCount = 0;
//		System.out.println("Auto zero initialize");
	}

	@Override
	protected void execute() {
		
	}

	@Override
	protected boolean isFinished() {
		Robot.arm.setSpeed(Arm.AUTO_ZERO_SPEED);
		double currentArmPosition = Robot.arm.getPositionInches();
		double armPositionChange = lastArmPosition - currentArmPosition;
		lastArmPosition = currentArmPosition;
		boolean test = encoderCount > 2 && Math.abs(armPositionChange) < MIN_ELEVATOR_POSITION_CHANGE && Robot.arm.getAverageMotorCurrent() > Arm.AUTO_ZERO_MOTOR_CURRENT;
		System.out.println("encoderCount = " + encoderCount + ", test = " + test + ", arm change = " + armPositionChange + ", current = " + Robot.arm.getAverageMotorCurrent());
		
		if (Math.abs(armPositionChange) < MIN_ELEVATOR_POSITION_CHANGE) {
			encoderCount++;
		}
		else {
			encoderCount = 0;
		}
		
		return test;
	}

	@Override
	protected void end() {
		Robot.arm.setSpeed(0);
		Robot.arm.resetZeroPosition(Arm.ZERO_POSITION_INCHES);
		Robot.arm.setPositionPID(Arm.MIN_POSITION_ROT);
//		System.out.println("Arm Zeroed");
	}

	@Override
	protected void interrupted() {
			
	}
}
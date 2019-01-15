package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import edu.wpi.first.wpilibj.command.Command;

public class DriveAbsoluteTurnMP extends Command
{
	private double absoluteTurnAngleDeg, maxTurnRateDegPerSec;
	private MPSoftwareTurnType turnType;

	public DriveAbsoluteTurnMP(double absoluteTurnAngleDeg, double maxTurnRateDegPerSec, MPSoftwareTurnType turnType) {
		requires(Robot.drive);
		this.absoluteTurnAngleDeg = absoluteTurnAngleDeg;
		this.maxTurnRateDegPerSec = maxTurnRateDegPerSec;
		this.turnType = turnType;
	}

	protected void initialize() {
//		if (Robot.drive.isRed() == false) {
//			absoluteTurnAngleDeg = absoluteTurnAngleDeg * -1;
//		}
		Robot.drive.setAbsoluteTurnMP(absoluteTurnAngleDeg, maxTurnRateDegPerSec, turnType);
	}

	protected void execute() {
	}

	protected boolean isFinished() {
		return Robot.drive.isFinished(); 
	}

	protected void end() {
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
		end();
	}
}
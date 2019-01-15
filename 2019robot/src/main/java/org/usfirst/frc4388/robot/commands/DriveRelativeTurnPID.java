package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import edu.wpi.first.wpilibj.command.Command;

public class DriveRelativeTurnPID extends Command
{
	private double relativeTurnAngleDeg;
	private MPSoftwareTurnType turnType;

	public DriveRelativeTurnPID(double relativeTurnAngleDeg, MPSoftwareTurnType turnType) {
		requires(Robot.drive);
		this.relativeTurnAngleDeg = relativeTurnAngleDeg;
		this.turnType = turnType;
	}

	protected void initialize() {
		Robot.drive.setRelativeTurnPID(relativeTurnAngleDeg, 0.3, 0.1, turnType);
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
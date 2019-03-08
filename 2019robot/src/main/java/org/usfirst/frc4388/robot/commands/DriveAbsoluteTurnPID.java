/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import edu.wpi.first.wpilibj.command.Command;

public class DriveAbsoluteTurnPID extends Command {

  private double absoluteTurnAngleDeg;
	private MPSoftwareTurnType turnType;

  /**
   * @param absoluteTurnAngleDeg The angle on the NavX for the drive train to turn to
   * @param turnType The type of turn to get to the angle
   */
  public DriveAbsoluteTurnPID(double absoluteTurnAngleDeg, MPSoftwareTurnType turnType) {
    requires(Robot.drive);
    this.absoluteTurnAngleDeg = absoluteTurnAngleDeg;
		this.turnType = turnType;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.setRelativeTurnPID(absoluteTurnAngleDeg, 0.3, 0.1, turnType);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drive.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

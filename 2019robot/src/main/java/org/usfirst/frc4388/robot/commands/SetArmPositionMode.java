/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Arm.ArmPositionMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @param mode 
 */
public class SetArmPositionMode extends Command {
  public static ArmPositionMode mode;

  public SetArmPositionMode(ArmPositionMode mode) {
    requires(Robot.arm);
    this.mode = mode;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.setArmPositionMode(mode);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

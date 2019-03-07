/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Pneumatics;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Sets the hatch intake and ball intake to the boolean values passed in.
 * This will have no effect if called and both subsystems are already in the desired state.
 * @param hatch true is deployed, false is undeployed
 * @param ball true is deployed, false is undeployed
 */

public class HatchAndBallSet extends Command {

  private boolean hatchposition;
  private boolean ballposition;

  public HatchAndBallSet(boolean hatch, boolean ball) { 
    requires(Robot.pnumatics);
    hatchposition = !hatch;
    ballposition = ball;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.pnumatics.setBallIntake(ballposition);
    Robot.pnumatics.setHatchIntakeState(hatchposition);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
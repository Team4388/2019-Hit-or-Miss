/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Pneumatics;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class HatchAndBallSet extends CommandGroup {

  private boolean hatch;
  private boolean ball;

  /**
   * Sets the hatch intake and ball intake to the boolean values passed in.
   * This will have no effect if called and both subsystems are already in the desired state.
   * @param hatch true is deploy, false is undeploy
   * @param ball true is deploy, false is undeploy
   */
  public HatchAndBallSet(boolean hatch, boolean ball) { 
    requires(Robot.pnumatics);
    this.hatch = hatch;
    this.ball = ball;
    addSequential(new DeployHatchIntake(hatch));
    addParallel(new DeployBallIntake(ball));
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class LedSwitch extends Command {
  public LedSwitch() {
    requires(LED);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    float lastTime = System.currentTimeMillis;
    LED.mode1();
  }

  /** 
   * Called repeatedly when this Command is scheduled to run. Cycles through 3 LED modes 
   * before returning to begining
   */
  @Override
  protected void execute() {
    if(System.currentTimeMillis>lastTime+30000){ //After 30 seconds
      LED.mode1();
      float lastTime = System.currentTimeMillis;
    }
    else if(System.currentTimeMillis>lastTime+20000){//After 20 seconds
      LED.mode3();
    }
    else if( System.currentTimeMillis>lastTime+10000){//After 10 seconds
      LED.mode2();
    }

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

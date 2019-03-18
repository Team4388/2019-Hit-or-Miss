/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Arm;
import org.usfirst.frc4388.robot.subsystems.Arm.ArmControlMode;

import edu.wpi.first.wpilibj.command.Command;


public class togglemanuel extends Command {
  public togglemanuel() {
    System.err.print("togle");
    if (Robot.arm.getArmControlMode() == ArmControlMode.MOTION_MAGIC){
      new ArmSetMode(ArmControlMode.JOYSTICK_MANUAL);
      System.err.print("manuel");
    }
    else if (Robot.arm.getArmControlMode() == ArmControlMode.JOYSTICK_MANUAL){
      new ArmSetMode(ArmControlMode.MOTION_MAGIC);
      System.err.print("Smart control");
    }
    else{
      new ArmSetMode(ArmControlMode.JOYSTICK_MANUAL);
    }

    
  }
    

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.err.print("pls work");
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

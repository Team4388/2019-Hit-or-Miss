/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LaserToDash extends Command {
  String p1 = "";
  String p2 = "";

  String last1 = "0";
  String last2 = "0";

  public LaserToDash() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.rangefinder);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    try{
      String toParse = Robot.rangefinder.getDistance();
      String[] parts = toParse.split("s");
      p1 = parts[0];
      p2 = parts[1];
      if (!(p1 == "0" || p2 == "0")){
        SmartDashboard.putNumber("Laser 1 raw out", Double.parseDouble(p1));
        SmartDashboard.putNumber("Laser 2 raw out", Double.parseDouble(p2));
        last1 = p1;
        last2 = p2;
      }
      else{
        SmartDashboard.putNumber("Laser 1 raw out", Double.parseDouble(last1));
        SmartDashboard.putNumber("Laser 2 raw out", Double.parseDouble(last2));
      }
    }
    catch(Exception nullException){
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

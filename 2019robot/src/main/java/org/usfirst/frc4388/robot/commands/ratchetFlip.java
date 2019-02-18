package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Climber;
import org.usfirst.frc4388.robot.Constants;

import edu.wpi.first.wpilibj.command.Command;

public class ratchetFlip extends Command {
  private double speed;
  public ratchetFlip(double speed) {
    requires(Robot.climber);
    this.speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (Climber.SPEED == 0){
      Robot.climber.flipRatchet(speed);
    }
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

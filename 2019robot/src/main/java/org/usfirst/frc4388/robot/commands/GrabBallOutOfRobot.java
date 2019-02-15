/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
* Add your docs here.
*/

public class GrabBallOutOfRobot extends CommandGroup 
{
  public GrabBallOutOfRobot() 
  {
    //Add rest of sequentials for this command group

    addSequential(new FlipIntake());
    //Move Arm until bar jump angle 

    //Jump bar in robot
    if(Robot.wrist.armAngleDegrees <= Robot.wrist.jumpBarArmAngle && Robot.wrist.armAngleDegrees >= Robot.wrist.jumpBarArmAngle)
    {
      addParallel(new WaitCommand(2));
      addSequential(new RunWristMotorJumpBar());
      addSequential(new StopWristMotor());
    }

    //continue arm movement until ball is in intake ///SENSOR for ball in robot

    //Move ball out of robot to 360 degrees
      //move arm and wrist in parallel
  }
}

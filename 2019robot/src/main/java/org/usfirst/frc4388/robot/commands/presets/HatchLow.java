/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands.presets;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.commands.ArmSetPositionMM;
import org.usfirst.frc4388.robot.commands.HatchFlip;
import org.usfirst.frc4388.robot.commands.WristSetPositionPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class HatchLow extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HatchLow() {
    boolean HatchFlipStatus = Robot.pnumatics.hatchIntakeState;
    addSequential(new HatchFlip(false));
    addParallel(new WristSetPositionPID(450));
    addSequential(new ArmSetPositionMM(590));
    addSequential(new HatchFlip(HatchFlipStatus));

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}

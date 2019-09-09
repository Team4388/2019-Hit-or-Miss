/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands.presets;

import org.usfirst.frc4388.robot.commands.ArmSetPositionSM;
import org.usfirst.frc4388.robot.commands.HatchFlip;
import org.usfirst.frc4388.robot.commands.WristSetPositionPID;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoHigh extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CargoHigh() {
    addSequential(new HatchFlip(false));
    addParallel(new setWrist(2781));
    addParallel(new DelayHatch());
    addSequential(new ArmSetPositionSM(4038));

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

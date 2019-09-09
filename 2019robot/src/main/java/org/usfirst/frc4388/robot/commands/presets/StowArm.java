/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.commands.presets;

import org.usfirst.frc4388.robot.commands.ArmSetPositionSM;
import org.usfirst.frc4388.robot.commands.HatchFlip;
import org.usfirst.frc4388.robot.commands.WristPlacement;
import org.usfirst.frc4388.robot.commands.WristSetPositionPID;
import org.usfirst.frc4388.robot.commands.setLEDPattern;
import org.usfirst.frc4388.robot.constants.LEDPatterns;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class StowArm extends CommandGroup {
  /**
   * Add your docs here.
   */
  public StowArm() {
    addSequential(new HatchFlip(false));
    addParallel(new WristPlacement(true));
    addParallel(new setLEDPattern(LEDPatterns.SOLID_GREEN));
    addParallel(new WristSetPositionPID(110), 2);
    addSequential(new ArmSetPositionSM(10));
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

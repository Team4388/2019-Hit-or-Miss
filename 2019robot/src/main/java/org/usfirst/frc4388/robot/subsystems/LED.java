/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4388.robot.subsystems;

import java.util.HashMap;

import org.usfirst.frc4388.robot.constants.LEDPatterns;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls the 5v LED Strip
 */
 public class LED extends Subsystem {

  public static final int LED_SPARK_ID = 0;
  public static float currentLED;
  public static Spark LEDController = new Spark(LED_SPARK_ID);

  public LED(){
    setPattern(LEDPatterns.FOREST_WAVES);
    LEDController.set(currentLED);
  }

  public void periodic() {
    LEDController.set(currentLED);
    SmartDashboard.putNumber("LED", currentLED);
  }

  public void setPattern(LEDPatterns pattern){
    currentLED = pattern.getValue();
    LEDController.set(currentLED);
  }

  @Override
	public void initDefaultCommand() {
  }
}
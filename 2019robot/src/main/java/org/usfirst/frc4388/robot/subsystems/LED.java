package org.usfirst.frc4388.robot.subsystems;

import org.usfirst.frc.team2848.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class LED extends Subsystem {

	//LED Spark controller
	public Spark LED1 = new Spark(RobotMap.LED_DRIVER);
	

	//public void initDefaultCommand() {}
	
	//LED mode 1
	public void mode1() { ///TODO: name the modes
		LED1.set(-.57);
	}
	
	//LED mode 2
	public void mode2() {
		LED1.set(.32);
	}
	
	//LED mode 3
	public void mode3() {
		LED1.set(-.69);
    }
    
}
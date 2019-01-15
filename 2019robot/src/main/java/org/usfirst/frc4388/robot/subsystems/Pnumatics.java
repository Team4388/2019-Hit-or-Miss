package org.usfirst.frc4388.robot.subsystems;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Pnumatics extends Subsystem {
    
		
	private DoubleSolenoid speedShift;
	private DoubleSolenoid Intake;

	
	public Pnumatics() {
		try {
			speedShift = new DoubleSolenoid(1,0,1);					
			Intake = new DoubleSolenoid(1,4,5 ); 
		} 
		catch (Exception e) {
			System.err.println("An error occurred in the Pnumatics constructor");
		}
	}
	
	public void setShiftState(boolean state) {
		if (state==true) {
			speedShift.set(DoubleSolenoid.Value.kForward);
		}
		if (state==false) {
			speedShift.set(DoubleSolenoid.Value.kReverse);
		}
	}
	public void setIntake(boolean state) {
		if (state==true) {
			Intake.set(DoubleSolenoid.Value.kForward);
		}
		if (state==false) {
			Intake.set(DoubleSolenoid.Value.kReverse);
		}
	}

	public void initDefaultCommand() {
    }
}


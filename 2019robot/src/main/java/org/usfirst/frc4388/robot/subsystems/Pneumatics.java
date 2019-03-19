package org.usfirst.frc4388.robot.subsystems;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Pneumatics extends Subsystem {
    
		
	private DoubleSolenoid speedShift;
	private DoubleSolenoid hatchIntake;
	private DoubleSolenoid ballIntake;
	private DoubleSolenoid wrist;

	public boolean hatchIntakeState = false;
	public boolean ballIntakeState = false;
	public boolean speedShiftState = false;
	public boolean wristState = false;
	
	public Pneumatics() {
		try {
			speedShift = new DoubleSolenoid(1,0,1);					
			hatchIntake = new DoubleSolenoid(1,2,3); 
			ballIntake = new DoubleSolenoid(1,4,5);
			wrist = new DoubleSolenoid(1,6,7);
		} 
		catch (Exception e) {
			System.err.println("An error occurred in the Pneumatics constructor");
		}
	}
	
	public void setShiftState(boolean state) {
		if (state==true) {
			speedShift.set(DoubleSolenoid.Value.kForward);
		}
		if (state==false) {
			speedShift.set(DoubleSolenoid.Value.kReverse);
		}
		speedShiftState = state;
	}
	public void setHatchIntakeState(boolean state) {
		if (state==true) {
			hatchIntake.set(DoubleSolenoid.Value.kForward);
		}
		if (state==false) {
			hatchIntake.set(DoubleSolenoid.Value.kReverse);
		}
		hatchIntakeState = state;
	}
	public void setBallIntake(boolean state) {
		if (state==true) {
			ballIntake.set(DoubleSolenoid.Value.kForward);
		}
		if (state==false) {
			ballIntake.set(DoubleSolenoid.Value.kReverse);
		}
		ballIntakeState = state;
	}
	public void setWrist(boolean state) {
		if (state==true) {
			wrist.set(DoubleSolenoid.Value.kForward);
		}
		if (state==false) {
			wrist.set(DoubleSolenoid.Value.kReverse);
		}
		wristState = state;
	}

	public void initDefaultCommand() {
    }
}


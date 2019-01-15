



package org.usfirst.frc4388.robot.subsystems;

import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.RobotMap;
import org.usfirst.frc4388.robot.commands.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


/**
 *
 */
public class Climber extends Subsystem{
	
	private WPI_TalonSRX Climber;

	public boolean on;
	
	public Climber(){
		
		try{
			
			Climber = new WPI_TalonSRX(RobotMap.CLIMBER_CAN_ID);
			
		} catch (Exception e) {
			
			System.err.println("An error occurred in the climbing constructor");
			
		}
	}

    @Override
    public void initDefaultCommand() {

    }


    @Override
    public void periodic() {
        // Put code here to be run every loop

    }
    
	public void setClimbSpeed(boolean Climb) {
		if (Climb==true) {
			Climber.set(1.0);
		}
		if (Climb==false) {
			Climber.set(0);
		}
}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
 {
			// TODO Auto-generated method stub
			
		}

}


package org.usfirst.frc4388.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import org.usfirst.frc4388.controller.Robot.OperationMode;
import org.usfirst.frc4388.robot.commands.*;

import org.usfirst.frc4388.robot.OI;
import org.usfirst.frc4388.robot.subsystems.*;
import org.usfirst.frc4388.utility.ControlLooper;
import org.usfirst.frc4388.robot.subsystems.Drive;

import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;;

/**
 * Initialises all the subsystems and directs which processes to 
 * run during autonomous and periodic modes
 */
public class Robot extends TimedRobot
{

	public static OI oi;
	
	public static final Drive drive = new Drive();
	public static final Arm arm = new Arm();
	public static final Wrist wrist = new Wrist();
	public static final BallIntake ballIntake = new BallIntake();
    public static final Climber climber = new Climber();
	public static final Pneumatics pnumatics = new Pneumatics();
	public static final LED led = new LED();

	public static final long periodMS = 10;
	public static final ControlLooper controlLoop = new ControlLooper("Main control loop", periodMS);

	public static enum OperationMode { TEST, COMPETITION };
	public static OperationMode operationMode = OperationMode.TEST;

	private SendableChooser<OperationMode> operationModeChooser;
    public void robotInit() 
    {
    	//Printing game data to riolog
    	String gameData = DriverStation.getInstance().getGameSpecificMessage();
    	System.out.println(gameData);
    	//CameraServer.getInstance().startAutomaticCapture();
    	//CameraServer.getInstance().putVideo("res", 300, 220);
    	
      try {
		oi = OI.getInstance();
		
		controlLoop.addLoopable(drive);
		controlLoop.addLoopable(arm);
		controlLoop.addLoopable(wrist);
			

        operationModeChooser = new SendableChooser<OperationMode>();
	    operationModeChooser.addDefault("Test", OperationMode.TEST);
	    operationModeChooser.addObject("Competition", OperationMode.COMPETITION);
		SmartDashboard.putData("Operation Mode", operationModeChooser);
		
		
		
		
		
		

		
		//ledLights.setAllLightsOn(false);
      } catch (Exception e) {
    		System.err.println("An error occurred in robotInit()");
      }
    }
	
    public void disabledInit(){

    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateStatus();
	}

    public void autonomousInit() {    	
    	updateChoosers();
    	controlLoop.start();
		drive.endGyroCalibration();
		arm.targetPositionInchesMM = arm.motor1.getPositionWorld();
		wrist.targetPositionInchesMM = wrist.wristMotor1.getPositionWorld();
    	//drive.resetEncoders();
    	//drive.resetGyro();
		//drive.setIsRed(getAlliance().equals(Alliance.Red));
		//arm.resetEncoder();
        updateStatus();
	}
     
    public void autonomousPeriodic() {
    	Scheduler.getInstance().run();
		updateStatus();
    }

    public void teleopInit() {
//        drive.setToBrakeOnNeutral(false);	
		updateChoosers();
		controlLoop.start();
		//drive.resetEncoders();
		drive.endGyroCalibration();
		arm.targetPositionInchesMM = arm.motor1.getPositionWorld();
		wrist.targetPositionInchesMM = wrist.wristMotor1.getPositionWorld();
        updateStatus();
    }


    public void teleopPeriodic() 
    {
        Scheduler.getInstance().run();
		updateStatus();
    }
    
    public void testPeriodic() {
        LiveWindow.run();
		updateStatus();
   }
    
    private void updateChoosers() {
		operationMode = (OperationMode)operationModeChooser.getSelected();
    }
    
    public Alliance getAlliance() {
    	return m_ds.getAlliance();
    }
    
    public void updateStatus() {
		arm.updateStatus(operationMode);
		drive.updateStatus(operationMode);
		wrist.updateStatus(operationMode);

   }

}


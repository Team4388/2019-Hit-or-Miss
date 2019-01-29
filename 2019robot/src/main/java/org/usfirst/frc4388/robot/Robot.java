
package org.usfirst.frc4388.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
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
import org.usfirst.frc4388.robot.subsystems.Arm;

import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;;

public class Robot extends IterativeRobot 
{

	public static OI oi;
	
	public static final Drive drive = new Drive();
    public static final Arm arm = new Arm();
    public static final Climber climber = new Climber();
	public static final Pnumatics pnumatics = new Pnumatics();
	public static final Wrist wrist = new Wrist();
	public static final long periodMS = 10;
	public static final ControlLooper controlLoop = new ControlLooper("Main control loop", periodMS);

	public static enum OperationMode { TEST, COMPETITION };
	public static OperationMode operationMode = OperationMode.TEST;

	private SendableChooser<OperationMode> operationModeChooser;
	private SendableChooser<Command> RRautonTaskChooser;
	private SendableChooser<Command> RLautonTaskChooser;
	private SendableChooser<Command> LRautonTaskChooser;
	private SendableChooser<Command> LLautonTaskChooser;

    private Command RRautonomousCommand;
    private Command RLautonomousCommand;
    private Command LRautonomousCommand;
	private Command LLautonomousCommand;

    public void robotInit() 
    {
    	//Printing game data to riolog
    	String gameData = DriverStation.getInstance().getGameSpecificMessage();
    	System.out.println(gameData);
    	CameraServer.getInstance().startAutomaticCapture();
    	CameraServer.getInstance().putVideo("res", 300, 220);
    	
      try {
		oi = OI.getInstance();
		
    	controlLoop.addLoopable(drive);
    	//controlLoop.addLoopable(arm);
			

        operationModeChooser = new SendableChooser<OperationMode>();
	    operationModeChooser.addDefault("Test", OperationMode.TEST);
	    operationModeChooser.addObject("Competition", OperationMode.COMPETITION);
		SmartDashboard.putData("Operation Mode", operationModeChooser);

		//Initializing the angle of the arm to be 
		arm.setInitAngle();

		//ledLights.setAllLightsOn(false);
	  } 
	  catch (Exception e) 
	  {
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
    	drive.resetEncoders();
    	drive.resetGyro();
    	drive.setIsRed(getAlliance().equals(Alliance.Red));
        
	}
 

    
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
		updateStatus();
    }

    public void teleopInit() {
        if (RRautonomousCommand != null) RRautonomousCommand.cancel();
        if (RLautonomousCommand != null) RLautonomousCommand.cancel();
        if (LRautonomousCommand != null) LRautonomousCommand.cancel();
        if (LLautonomousCommand != null) LLautonomousCommand.cancel();
        drive.setToBrakeOnNeutral(false);	
    	updateChoosers();
        controlLoop.start();
    	drive.resetEncoders();
    	drive.endGyroCalibration();

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
		RRautonomousCommand = (Command)RRautonTaskChooser.getSelected();
		RLautonomousCommand = (Command)RLautonTaskChooser.getSelected();
		LRautonomousCommand = (Command)LRautonTaskChooser.getSelected();
		LLautonomousCommand = (Command)LLautonTaskChooser.getSelected();
    }
    
	public Alliance getAlliance() 
	{
    	return m_ds.getAlliance();
    }
    
	public void updateStatus() 
	{
    	drive.updateStatus(operationMode);
    	arm.updateStatus(operationMode);
   }

}


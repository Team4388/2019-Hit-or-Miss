
package org.usfirst.frc4388.robot;

import java.util.HashMap;
//import org.usfirst.frc4388.controller.Robot.OperationMode;
import org.usfirst.frc4388.robot.commands.ArmAutoZero;//
import org.usfirst.frc4388.robot.Robot.OperationMode;
import org.usfirst.frc4388.robot.subsystems.Arm;
import org.usfirst.frc4388.robot.subsystems.Climber;
import org.usfirst.frc4388.robot.subsystems.Drive;
import org.usfirst.frc4388.robot.subsystems.Pnumatics;
import org.usfirst.frc4388.robot.subsystems.Wrist;
//import org.usfirst.frc4388.utility.ControlLooper;
import org.usfirst.frc4388.utility.Looper;
import org.usfirst.frc4388.utility.control.RobotStateEstimator;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.control.RobotState;

import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;

public class Robot extends TimedRobot
{

	public static OI oi;

	public static final Drive drive = Drive.getInstance();
	public static final Arm arm = Arm.getInstance();
	

	public static final Climber climber = new Climber();
	public static final Pnumatics pnumatics = new Pnumatics();
	public static final Wrist wrist = new Wrist();
	public static final long periodMS = 10;
	// public static final ControlLooper controlLoop = new ControlLooper("Main
	// control loop", periodMS);

	public static final Looper controlLoop = new Looper();
	private SendableChooser<OperationMode> operationModeChooser;
	
    private Command autonomousCommand;

	public static enum OperationMode { TEST, PRACTICE, COMPETITION };
	public static OperationMode operationMode = OperationMode.COMPETITION;


	private RobotState robotState =/* org.usfirst.frc4388.utility.control.*/RobotState.getInstance();

	public void zeroAllSensors() {
		//drive.zeroSensors();
		robotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
		//drive.zeroSensors();
		}





	@Override
    public void robotInit() 
    {
    	
   		setPeriod(Constants.kLooperDt * 2);
		System.out.println("Main loop period = " + getPeriod());
		oi = OI.getInstance();
		controlLoop.register(drive);
		controlLoop.register(arm);
		controlLoop.register(RobotStateEstimator.getInstance());
	
    	operationModeChooser = new SendableChooser<OperationMode>();
	    operationModeChooser.addDefault("Competition", OperationMode.COMPETITION);
	    operationModeChooser.addObject("Test", OperationMode.TEST);
		SmartDashboard.putData("Operation Mode", operationModeChooser);
		LiveWindow.setEnabled(false);
		zeroAllSensors();

    }
	
	private void setPeriod(double d) {
	}

	public void robotPeriodic() {
		updateStatus();
	}
 
	@Override
	public void disabledInit() {
		drive.setLimeLED(false);
    }
	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateStatus();
	}
	@Override
    public void autonomousInit() {  
		controlLoop.start();
    	drive.setIsRed(getAlliance().equals(Alliance.Red));
    	arm.resetZeroPosition(Arm.ZERO_POSITION_INCHES);
    	updateChoosers();
    	drive.endGyroCalibration();
    	drive.resetEncoders();
    	drive.resetGyro();
		drive.setIsRed(getAlliance().equals(Alliance.Red));
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}	
        
	}
 

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
		updateStatus();
    }
	@Override
    public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		operationMode = operationModeChooser.getSelected();
		
        controlLoop.start();
    	//drive.setShiftState(DriveSpeedShiftState.LO);
    	drive.endGyroCalibration();
        zeroAllSensors();

    	if (operationMode != OperationMode.COMPETITION) {
    		Scheduler.getInstance().add(new ArmAutoZero(false));
    	}
    	else {
            arm.setPositionPID(arm.getPositionInches());
    	}
	}

		//Arm.setArmControlMode(Arm.ArmControlMode.JOYSTICK_MANUAL);

	@Override
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
    
	public Alliance getAlliance() 
	{
    	return m_ds.getAlliance();
    }
    public double getMatchTime() {
    	return m_ds.getMatchTime();
	}
	
	public void updateStatus() 
	{
    	drive.updateStatus(operationMode);
		arm.updateStatus(operationMode);
		robotState.updateStatus(operationMode);

   }

}


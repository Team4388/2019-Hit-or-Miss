package org.usfirst.frc4388.robot;

import buttons.XBoxTriggerButton;
import org.usfirst.frc4388.controller.IHandController;
import org.usfirst.frc4388.controller.XboxController;
import org.usfirst.frc4388.robot.commands.*;
import org.usfirst.frc4388.robot.commands.presets.StowArm;
import org.usfirst.frc4388.robot.constants.LEDPatterns;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc4388.robot.subsystems.*;
import org.usfirst.frc4388.robot.subsystems.Arm.ArmControlMode;
import org.usfirst.frc4388.robot.subsystems.Arm.PlaceMode;
import org.usfirst.frc4388.robot.subsystems.Wrist.WristControlMode;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;


/**
 * Maps controler buttons to specific commands. Any controller will work but we 
 * use Xbox One controllers for our naming scheme.
 */
public class OI
{
		private static OI instance;

		private XboxController m_driverXbox;
		private XboxController m_operatorXbox;

		private OI()
		{
		  try
		  {
			// Driver controllers
			m_driverXbox = new XboxController(RobotMap.DRIVER_JOYSTICK_1_USB_ID);
			m_operatorXbox = new XboxController(RobotMap.OPERATOR_JOYSTICK_1_USB_ID);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			operator controller



	        XBoxTriggerButton CarriageIntake = new XBoxTriggerButton(m_operatorXbox, XBoxTriggerButton.LEFT_TRIGGER);
	        CarriageIntake.whenPressed(new SetIntakeSpeed(BallIntake.BALL_INTAKE_SPEED));
	        CarriageIntake.whenReleased(new SetIntakeSpeed(0.0));

	        XBoxTriggerButton CarriageEject = new XBoxTriggerButton(m_operatorXbox, XBoxTriggerButton.RIGHT_TRIGGER);
	        CarriageEject.whenPressed(new SetIntakeSpeed(BallIntake.BALL_EXTAKE_SPEED));
			CarriageEject.whenReleased(new SetIntakeSpeed(0.0));

			JoystickButton Expand = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.Y_BUTTON);
			Expand.whenPressed(new WristPlacement(true));
			Expand.whenPressed(new setLEDPattern(LEDPatterns.SOLID_GREEN));

			JoystickButton Contract = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.A_BUTTON);
			Contract.whenPressed(new WristPlacement(false));
			Contract.whenPressed(new setLEDPattern(LEDPatterns.SOLID_RED));

			JoystickButton liftBothIntake = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.X_BUTTON);
			liftBothIntake.whenPressed(new HatchAndBallUp());
			
			JoystickButton cargoPlaceMode = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.B_BUTTON);
			cargoPlaceMode.whenPressed(new SwitchArmPlaceMode(PlaceMode.CARGO));
			cargoPlaceMode.whenReleased(new SwitchArmPlaceMode(PlaceMode.HATCH));

			JoystickButton liftHatchIntake = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.RIGHT_BUMPER_BUTTON);
			liftHatchIntake.whenPressed(new LiftHatchDropBall());

			JoystickButton liftBallIntake = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.LEFT_BUMPER_BUTTON);
			liftBallIntake.whenPressed(new LiftBallDropHatch());

			JoystickButton safteySwitch = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.START_BUTTON);
			safteySwitch.whenPressed(new setClimberSafety(true));
			safteySwitch.whenPressed(new setLEDPattern(LEDPatterns.SOLID_YELLOW));
			safteySwitch.whenReleased(new setClimberSafety(false));
			safteySwitch.whenReleased(new setLEDPattern(LEDPatterns.FOREST_WAVES));

			JoystickButton Height1 = new JoystickButton(m_operatorXbox.getJoyStick(),XboxController.RIGHT_JOYSTICK_BUTTON);
			Height1.whenPressed(new ArmToHeight1());

			JoystickButton lowHeight = new JoystickButton(m_operatorXbox.getJoyStick(),XboxController.LEFT_JOYSTICK_BUTTON);
			lowHeight.whenPressed(new GrabFromLoadingSatation());

			//JoystickButton stow = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.B_BUTTON);
			//stow.whenPressed(new StowArm());
			//stow.whenPressed(new setLEDPattern(LEDPatterns.SOLID_GREEN));


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			Driver Xbox Controler


			JoystickButton climbUp = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.RIGHT_TRIGGER_AXIS);
			
			JoystickButton climbDown = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.LEFT_TRIGGER_AXIS);

	        JoystickButton shiftUp = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.RIGHT_BUMPER_BUTTON);
	        shiftUp.whenPressed(new DriveSpeedShift(true));

	        JoystickButton shiftDown = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.LEFT_BUMPER_BUTTON);
			shiftDown.whenPressed(new DriveSpeedShift(false));
			
			JoystickButton help = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.X_BUTTON);
			help.whenPressed(new ArmSetMode(ArmControlMode.JOYSTICK_MANUAL));
			help.whenReleased(new ArmSetMode(ArmControlMode.MOTION_MAGIC));
			help.whenPressed(new WristSetMode(WristControlMode.JOYSTICK_MANUAL));
			help.whenReleased(new WristSetMode(WristControlMode.JOYSTICK_PID));

			JoystickButton panic = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.START_BUTTON);
			panic.whenPressed(new ResetArmEncoder());
			panic.whenPressed(new ResetWristEncoder());


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			OI buttons


			SmartDashboard.putData("switch to manuel", new SetManual());
			SmartDashboard.putData("WristTest", new wristTest());
			SmartDashboard.putData("arm test", new ArmTest());
			//SmartDashboard.putData("arm test2", new ArmTest2());

			

		  } catch (Exception e) {
			  System.err.println("An error occurred in the OI constructor");
		  }
		}

		public static OI getInstance() {
			if(instance == null) {
				instance = new OI();
			}
			return instance;
		}

		public IHandController getDriverController() {
			return m_driverXbox;
		}

		public IHandController getOperatorController()
		{
			return m_operatorXbox;
		}

		public Joystick getOperatorJoystick()
		{
			return m_operatorXbox.getJoyStick();
		}
	}

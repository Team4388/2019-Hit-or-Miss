


package org.usfirst.frc4388.robot;

import buttons.XBoxTriggerButton;
import org.usfirst.frc4388.controller.IHandController;
import org.usfirst.frc4388.controller.XboxController;
import org.usfirst.frc4388.robot.commands.*;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc4388.robot.subsystems.*;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;
import org.usfirst.frc4388.robot.subsystems.Drive;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.InternalButton;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;



public class OI
{
		private static OI instance;

		private XboxController m_driverXbox;
		private XboxController m_operatorXbox;

		private OI()
		{
		  try
		  {
			// Driver controller
			m_driverXbox = new XboxController(RobotMap.DRIVER_JOYSTICK_1_USB_ID);
			m_operatorXbox = new XboxController(RobotMap.OPERATOR_JOYSTICK_1_USB_ID);

	        XBoxTriggerButton CarriageIntake = new XBoxTriggerButton(m_operatorXbox, XBoxTriggerButton.LEFT_TRIGGER);
	        CarriageIntake.whenPressed(new SetIntakeSpeed(BallIntake.BALL_INTAKE_SPEED));
	        CarriageIntake.whenReleased(new SetIntakeSpeed(0.0));

	        XBoxTriggerButton CarriageEject = new XBoxTriggerButton(m_operatorXbox, XBoxTriggerButton.RIGHT_TRIGGER);
	        CarriageEject.whenPressed(new SetIntakeSpeed(BallIntake.BALL_EXTAKE_SPEED));
			CarriageEject.whenReleased(new SetIntakeSpeed(0.0));

			JoystickButton Expand = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.Y_BUTTON);
			Expand.whenPressed(new WristPlacement(true));

			JoystickButton Contract = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.A_BUTTON);
	        Contract.whenPressed(new WristPlacement(false));


			JoystickButton liftBothIntake = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.X_BUTTON);
			liftBothIntake.whenPressed(new HatchAndBallSet(false, false));

			JoystickButton liftHatchIntake = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.RIGHT_BUMPER_BUTTON);
			liftHatchIntake.whenPressed(new HatchAndBallSet(false, true));


			JoystickButton liftBallIntake = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.LEFT_BUMPER_BUTTON);
			//liftBallIntake.whenPressed(new HatchFlip(false));
			liftBallIntake.whenPressed(new HatchAndBallSet(true, false));


			JoystickButton climbUp = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.RIGHT_TRIGGER_AXIS);
			JoystickButton climbDown = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.LEFT_TRIGGER_AXIS);

			JoystickButton ratchetFlip = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.Y_BUTTON);
			ratchetFlip.whenPressed(new ratchetFlip(0.5));
			ratchetFlip.whenReleased(new ratchetFlip(0));

	        JoystickButton shiftUp = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.RIGHT_BUMPER_BUTTON);
	        shiftUp.whenPressed(new DriveSpeedShift(true));
	        //shiftUp.whenPressed(new LEDIndicators(true));

	        JoystickButton shiftDown = new JoystickButton(m_driverXbox.getJoyStick(), XboxController.LEFT_BUMPER_BUTTON);
	        shiftDown.whenPressed(new DriveSpeedShift(false));
	       // shiftDown.whenPressed(new LEDIndicators(false));
	        //Operator Xbox
			/*
	        JoystickButton openIntake = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.LEFT_BUMPER_BUTTON);
			openIntake.whenPressed(new IntakePosition(true));
			s

	        JoystickButton CloseIntake = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.RIGHT_BUMPER_BUTTON);
			CloseIntake.whenPressed(new IntakePosition(false));
			*/

			SmartDashboard.putData("switch to manuel", new SetManual());
			SmartDashboard.putData("run arm test", new ArmTest());

			JoystickButton safteySwitch = new JoystickButton(m_operatorXbox.getJoyStick(), XboxController.START_BUTTON);
			safteySwitch.whenPressed(new setClimberSafety(true));
			safteySwitch.whenReleased(new setClimberSafety(false));

	        //SmartDashboard.putData("PRE GAME!!!!", new PreGame());
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

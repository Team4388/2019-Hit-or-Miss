package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Constants;
import org.usfirst.frc4388.robot.Robot;
import org.usfirst.frc4388.robot.subsystems.Drive.DriveControlMode;
import org.usfirst.frc4388.utility.BHRMathUtils;
import org.usfirst.frc4388.utility.CANTalonEncoder;
import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;

public class DriveTurnBasic extends Command
{
	private boolean m_useAbsolute;			// if true, we don't reset the gyro before starting the turn
	private double m_turnAngleDeg;			// either absolute or relative to current heading, depending on useAbsolute
	private double m_maxTurnRateDegPerSec;
	private MPSoftwareTurnType m_turnType;
	private boolean m_turningLeft;
	private double m_targetGyroAngleDeg;	// what we want the gyro to read when we're done

	public DriveTurnBasic(boolean useAbsolute, double turnAngleDeg, double maxTurnRateDegPerSec, MPSoftwareTurnType turnType) {
		requires(Robot.drive);
		m_useAbsolute = useAbsolute;
		m_turnAngleDeg = turnAngleDeg;
		m_maxTurnRateDegPerSec = maxTurnRateDegPerSec;
		m_turnType = turnType;
	}

	protected void initialize() {
		double currentAngleDeg = Robot.drive.getGyroAngleDeg();
		if (m_useAbsolute) {
			m_targetGyroAngleDeg = m_turnAngleDeg;
		} else {
			m_targetGyroAngleDeg = currentAngleDeg + m_turnAngleDeg;
		}
		if ((m_turnType == MPSoftwareTurnType.RIGHT_SIDE_ONLY) || (m_turnType == MPSoftwareTurnType.RIGHT_ARC)) {
			m_turningLeft = true;	// gyro angle will be DEcreasing, so make sure we aim for an angle in that direction
			while (m_targetGyroAngleDeg >= currentAngleDeg) {
				m_targetGyroAngleDeg = m_targetGyroAngleDeg - 360.0;
			}
		} else if ((m_turnType == MPSoftwareTurnType.LEFT_SIDE_ONLY) || (m_turnType == MPSoftwareTurnType.LEFT_ARC)) {
			m_turningLeft = false;	// gyro angle will be INcreasing, so make sure we aim for an angle in that direction
			while (m_targetGyroAngleDeg <= currentAngleDeg) {
				m_targetGyroAngleDeg = m_targetGyroAngleDeg + 360.0;
			}
		} else {	// MPSoftwareTurnType.TANK -- need to determine based on values passed
			if (m_useAbsolute) {	// this is the only case where we have to DECIDE which direction to turn
				m_turnAngleDeg = BHRMathUtils.normalizeAngle0To360(m_turnAngleDeg);
				m_targetGyroAngleDeg = BHRMathUtils.adjustAccumAngleToDesired(currentAngleDeg, m_turnAngleDeg);
				m_turningLeft = (m_targetGyroAngleDeg < currentAngleDeg);
			} else {
				m_turningLeft = (m_turnAngleDeg < 0);
			}
		}
		System.out.println("Turning " + (m_turningLeft?"left":"right") + " from " + currentAngleDeg + " to " + m_targetGyroAngleDeg + " degrees");
    	Robot.drive.setControlMode(DriveControlMode.RAW);
    	Robot.drive.resetEncoders();
	}

	protected void execute() {
		double output = Constants.kDriveTurnBasicSingleMotorOutput;
		
		if (m_turnType == MPSoftwareTurnType.TANK) {
			output = Constants.kDriveTurnBasicTankMotorOutput;
			if (m_turningLeft) {
				Robot.drive.rawDriveLeftRight(-output, -output);	// left backward, right forward
			} else {
				Robot.drive.rawDriveLeftRight(output, output);		// left forward, right backward
			}
//			for (CANTalonEncoder motorController : motorControllers) {
//				//motorController.set(output);
//				motorController.set(ControlMode.PercentOutput, output);
//			}
		}
		else if (m_turnType == MPSoftwareTurnType.LEFT_SIDE_ONLY) {
			Robot.drive.rawDriveLeftRight(2.0 * output, 0.0);		// left forward double speed
//			for (CANTalonEncoder motorController : motorControllers) {
//				if (motorController.isRight()) {
//					//motorController.set(0);
//					motorController.set(ControlMode.PercentOutput, 0);
//				}
//				else {
//					//motorController.set(2.0 * output);					
//					motorController.set(ControlMode.PercentOutput, 2.0 * output);
//				}
//			}
		}
		else if (m_turnType == MPSoftwareTurnType.RIGHT_SIDE_ONLY) {
			Robot.drive.rawDriveLeftRight(0.0, -2.0 * output);		// right forward double speed
//			for (CANTalonEncoder motorController : motorControllers) {
//				if (motorController.isRight()) {
//					//motorController.set(2.0 * output);
//					motorController.set(ControlMode.PercentOutput, 2.0 * output);
//				}
//				else {
//					//motorController.set(0);					
//					motorController.set(ControlMode.PercentOutput, 0);
//				}
//			}
		}
		else if (m_turnType == MPSoftwareTurnType.LEFT_ARC) {
			Robot.drive.rawDriveLeftRight(2.0 * output, -output);	// left fwd 2x, right fwd 1x
//			for (CANTalonEncoder motorController : motorControllers) {
//				if (motorController.isRight()) {
//					//motorController.set(1.0 * output);
//					motorController.set(ControlMode.PercentOutput, 1.0 * output);
//				}
//				else {
//					//motorController.set(2.0 * output);					
//					motorController.set(ControlMode.PercentOutput, 2.0 * output);					
//				}
//			}
		}
		else if (m_turnType == MPSoftwareTurnType.RIGHT_ARC) {
			Robot.drive.rawDriveLeftRight(output, -2.0 * output);	// left fwd 1x, right fwd 2x
//			for (CANTalonEncoder motorController : motorControllers) {
//				if (motorController.isRight()) {
//					//motorController.set(2.0 * output);
//					motorController.set(ControlMode.PercentOutput, 2.0 * output);
//				}
//				else {
//					//motorController.set(1.0 * output);					
//					motorController.set(ControlMode.PercentOutput, 1.0 * output);					
//				}
//			}
		}
	}

	protected boolean isFinished() {
		boolean finished;
		double currentAngleDeg = Robot.drive.getGyroAngleDeg();
		
		if (m_turningLeft) {
			finished = currentAngleDeg <= m_targetGyroAngleDeg;
		} else {
			finished = currentAngleDeg >= m_targetGyroAngleDeg;
		}
		return finished;
	}

	protected void end() {
		Robot.drive.rawDriveLeftRight(0.0, 0.0);
		Robot.drive.setControlMode(DriveControlMode.JOYSTICK);
	}

	protected void interrupted() {
		end();
	}
}
package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc4388.utility.MPSoftwarePIDController.MPSoftwareTurnType;

public class DriveSetAngle extends Command {
    
    private double targetAngle;
    private double maxError;
    private double maxPrevError;
    private MPSoftwareTurnType turnType;
    // Called just before this Command runs the first time
    protected void initialize() {
    	turnToAngle(targetAngle, maxError, maxPrevError, turnType);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }
}

package org.usfirst.frc4388.robot.commands;

import org.usfirst.frc4388.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WristSetPositionMM extends Command {
	
	private double targetPositionInches;
	private boolean isAtTarget;
	private static final double MIN_DELTA_TARGET = 20;

    public WristSetPositionMM(double targetPositionInches) {
    	this.targetPositionInches = targetPositionInches;
        requires(Robot.wrist);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
      
    	if (Math.abs(targetPositionInches - Robot.wrist.getPositionInches()) < MIN_DELTA_TARGET) {
    		isAtTarget = true;
    	}
    	else {
        	isAtTarget = false;
        	Robot.wrist.setPositionMM(targetPositionInches);
    	}
//    	System.out.println("Arm set MP initialized, target = " + targetPositionInches);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isAtTarget || Robot.wrist.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.wrist.setPositionMM(Robot.wrist.getPositionInches());
//    	System.out.println("Arm set MP end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
//    	System.out.println("ArmSetPositionMP interrupted");
    	Robot.wrist.setFinished(true);
		Robot.wrist.setPositionMM(Robot.wrist.getPositionInches());
    }
}

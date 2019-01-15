package org.usfirst.frc4388.utility;

import java.util.ArrayList;
import java.util.Optional;
import java.util.Set;

import org.usfirst.frc4388.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4
 * .pdf
 * 
 * Basically, we find a spot on the path we'd like to follow and calculate the
 * wheel speeds necessary to make us land on that spot. The target spot is a
 * specified distance ahead of us, and we look further ahead the greater our
 * tracking error.
 */
public class AdaptivePurePursuitController {
    private static final double kEpsilon = 1E-9;

    double mFixedLookahead;
    Path mPath;
    RigidTransform2d.Delta mLastCommand;
    double mLastTime;
    double mMaxAccel;
    double mDt;
    boolean mReversed;
    double mPathCompletionTolerance;

	protected ArrayList<CANTalonEncoder> motorControllers;	
	protected long periodMs;
	protected PIDParams pidParams;	
	protected double startGyroAngle;
	protected double targetGyroAngle;
	protected double trackDistance;
	
	public AdaptivePurePursuitController(long periodMs, PIDParams pidParams, ArrayList<CANTalonEncoder> motorControllers) 
	{
		this.motorControllers = motorControllers;
		this.periodMs = periodMs;
		setPID(pidParams);
	}
    
	public void setPID(PIDParams pidParams) {
		this.pidParams = pidParams;
		
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPID(pidParams.kP, pidParams.kI, pidParams.kD);
			//motorController.setF(pidParams.kF);
			//motorController.configNominalOutputVoltage(+0.0f, -0.0f);
			//motorController.configPeakOutputVoltage(+12.0f, -12.0f);
			//motorController.setProfile(0);
			motorController.config_kP(0, pidParams.kP, 0);
			motorController.config_kI(0, pidParams.kI, 0);
			motorController.config_kD(0, pidParams.kD, 0);
			motorController.config_kF(0, pidParams.kF, 0);
			motorController.configNominalOutputForward(+0.0f, 0);
			motorController.configNominalOutputReverse(-0.0f, 0);
			motorController.configPeakOutputForward(+1.0f, 0);
			motorController.configPeakOutputReverse(-1.0f, 0);
			motorController.selectProfileSlot(0, 0);
		}
	}
	
    public void setPath(double fixed_lookahead, double max_accel, Path path,
            boolean reversed, double path_completion_tolerance) {
        mFixedLookahead = fixed_lookahead;
        mMaxAccel = max_accel;
        mPath = path;
        mDt = periodMs;
        mLastCommand = null;
        mReversed = reversed;
        mPathCompletionTolerance = path_completion_tolerance;

        // Set up the motion profile 
		for (CANTalonEncoder motorController : motorControllers) {
			//motorController.setPosition(0);
			//motorController.set(0);
			//motorController.changeControlMode(TalonControlMode.Speed);
			motorController.setSelectedSensorPosition(0, 0, 0);	//TODO: verify want 0="Primary closed-loop", with no timeout
			motorController.set(ControlMode.Velocity, 0);
		}
    }

    public boolean isDone() {
        double remainingLength = mPath.getRemainingLength();
        return remainingLength <= mPathCompletionTolerance;
    }

	public boolean controlLoopUpdate(RigidTransform2d robot_pose) {
        RigidTransform2d.Delta command = update(robot_pose, Timer.getFPGATimestamp());
        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

        // Scale the command to respect the max velocity limits
        double max_vel = 0.0;
        max_vel = Math.max(max_vel, Math.abs(setpoint.left));
        max_vel = Math.max(max_vel, Math.abs(setpoint.right));
        if (max_vel > Constants.kPathFollowingMaxVel) {
            double scaling = Constants.kPathFollowingMaxVel / max_vel;
            setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
        }

        // Update the controllers Kf and set point.
		for (CANTalonEncoder motorController : motorControllers) {
			if (motorController.isRight()) {
				motorController.setVelocityWorld(-setpoint.right);
			}
			else {
				motorController.setVelocityWorld(-setpoint.left);
			}
		}
		
		return isDone();
	}

	public RigidTransform2d.Delta update(RigidTransform2d robot_pose, double now) {
        RigidTransform2d pose = robot_pose;
        if (mReversed) {
            pose = new RigidTransform2d(robot_pose.getTranslation(),
                    robot_pose.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)));
        }

        double distance_from_path = mPath.update(robot_pose.getTranslation());
        if (this.isDone()) {
            return new RigidTransform2d.Delta(0, 0, 0);
        }

        PathSegment.Sample lookahead_point = mPath.getLookaheadPoint(robot_pose.getTranslation(),
                distance_from_path + mFixedLookahead);
        Optional<Circle> circle = joinPath(pose, lookahead_point.translation);

        double speed = lookahead_point.speed;
        if (mReversed) {
            speed *= -1;
        }
        // Ensure we don't accelerate too fast from the previous command
        double dt = now - mLastTime;
        if (mLastCommand == null) {
            mLastCommand = new RigidTransform2d.Delta(0, 0, 0);
            dt = mDt;
        }
        double accel = (speed - mLastCommand.dx) / dt;
        if (accel < -mMaxAccel) {
            speed = mLastCommand.dx - mMaxAccel * dt;
        } else if (accel > mMaxAccel) {
            speed = mLastCommand.dx + mMaxAccel * dt;
        }

        // Ensure we slow down in time to stop
        // vf^2 = v^2 + 2*a*d
        // 0 = v^2 + 2*a*d
        double remaining_distance = mPath.getRemainingLength();
        double max_allowed_speed = Math.sqrt(2 * mMaxAccel * remaining_distance);
        if (Math.abs(speed) > max_allowed_speed) {
            speed = max_allowed_speed * Math.signum(speed);
        }
        final double kMinSpeed = 4.0;
        if (Math.abs(speed) < kMinSpeed) {
            // Hack for dealing with problems tracking very low speeds with
            // Talons
            speed = kMinSpeed * Math.signum(speed);
        }

        RigidTransform2d.Delta rv;
        if (circle.isPresent()) {
            rv = new RigidTransform2d.Delta(speed, 0,
                    (circle.get().turn_right ? -1 : 1) * Math.abs(speed) / circle.get().radius);
        } else {
            rv = new RigidTransform2d.Delta(speed, 0, 0);
        }
        mLastTime = now;
        mLastCommand = rv;
        return rv;
    }

    public Set<String> getMarkersCrossed() {
        return mPath.getMarkersCrossed();
    }

    public static class Circle {
        public final Translation2d center;
        public final double radius;
        public final boolean turn_right;

        public Circle(Translation2d center, double radius, boolean turn_right) {
            this.center = center;
            this.radius = radius;
            this.turn_right = turn_right;
        }
    }

    public static Optional<Circle> joinPath(RigidTransform2d robot_pose, Translation2d lookahead_point) {
        double x1 = robot_pose.getTranslation().getX();
        double y1 = robot_pose.getTranslation().getY();
        double x2 = lookahead_point.getX();
        double y2 = lookahead_point.getY();

        Translation2d pose_to_lookahead = robot_pose.getTranslation().inverse().translateBy(lookahead_point);
        double cross_product = pose_to_lookahead.getX() * robot_pose.getRotation().sin()
                - pose_to_lookahead.getY() * robot_pose.getRotation().cos();
        if (Math.abs(cross_product) < kEpsilon) {
            return Optional.empty();
        }

        double dx = x1 - x2;
        double dy = y1 - y2;
        double my = (cross_product > 0 ? -1 : 1) * robot_pose.getRotation().cos();
        double mx = (cross_product > 0 ? 1 : -1) * robot_pose.getRotation().sin();

        double cross_term = mx * dx + my * dy;

        if (Math.abs(cross_term) < kEpsilon) {
            // Points are colinear
            return Optional.empty();
        }

        return Optional.of(new Circle(
                new Translation2d((mx * (x1 * x1 - x2 * x2 - dy * dy) + 2 * my * x1 * dy) / (2 * cross_term),
                        (-my * (-y1 * y1 + y2 * y2 + dx * dx) + 2 * mx * y1 * dx) / (2 * cross_term)),
                .5 * Math.abs((dx * dx + dy * dy) / cross_term), cross_product > 0));
    }

}

package org.usfirst.frc4388.utility;

import java.awt.Color;

import org.usfirst.frc4388.robot.subsystems.Drive;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class PathGenerator {
	
	private Segment[] centerPoints;
	private Segment[] leftPoints;
	private Segment[] rightPoints;
	private int curIndex = 0;
	
	public PathGenerator(Waypoint[] points, double timeStep, double maxVelocity, double maxAccel, double maxJerk) {

		boolean negativeFlag = false;
		for (int i = 0; i < points.length; i++) {
        	if (points[i].x < 0) {
        		negativeFlag = true;
        	}
        }
		if (negativeFlag == true) {
			for (int i = 0; i < points.length; i++) {
	        	points[i].x = -(points[i].x);
	        }
		}
		
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, timeStep, maxVelocity, maxAccel, maxJerk);
        Trajectory trajectory = Pathfinder.generate(points, config);
        centerPoints = trajectory.segments;       

	    TankModifier modifier = new TankModifier(trajectory).modify(Drive.TRACK_WIDTH_INCHES);
	    leftPoints = modifier.getLeftTrajectory().segments;
	    rightPoints = modifier.getRightTrajectory().segments; 

        if (negativeFlag == true) {
			for (int i = 0; i < centerPoints.length; i++) {
				Segment curSeg = centerPoints[i];
				curSeg.x = -(curSeg.x);
				curSeg.y = -(curSeg.y);
				curSeg.jerk = -(curSeg.jerk);
				curSeg.acceleration = -(curSeg.acceleration);
				curSeg.velocity = -(curSeg.velocity);
				curSeg.position = -(curSeg.position);
				curSeg.heading = -(curSeg.heading);
				curSeg = leftPoints[i];
				curSeg.x = -(curSeg.x);
				curSeg.y = -(curSeg.y);
				curSeg.jerk = -(curSeg.jerk);
				curSeg.acceleration = -(curSeg.acceleration);
				curSeg.velocity = -(curSeg.velocity);
				curSeg.position = -(curSeg.position);
				curSeg.heading = -(curSeg.heading);
				curSeg = rightPoints[i];
				curSeg.x = -(curSeg.x);
				curSeg.y = -(curSeg.y);
				curSeg.jerk = -(curSeg.jerk);
				curSeg.acceleration = -(curSeg.acceleration);
				curSeg.velocity = -(curSeg.velocity);
				curSeg.position = -(curSeg.position);
				curSeg.heading = -(curSeg.heading);
			}
		}
		
//      TankModifier2 modifier2 = new TankModifier2();
//      modifier2.modify(centerPoints, Drive.TRACK_WIDTH_INCHES);
//      leftPoints = modifier2.getLeftSegments();
//      rightPoints = modifier2.getRightSegments();
	}
	
	public Segment getLeftPoint() {
		return (curIndex < leftPoints.length) ? leftPoints[curIndex] : null;
	}
	
	public Segment getRightPoint() {
		return (curIndex < rightPoints.length) ? rightPoints[curIndex] : null;
	}
	
	public Segment getCenterPoint() {
		return (curIndex < centerPoints.length) ? centerPoints[curIndex] : null;
	}
	
	public Double getHeading() {
		if (curIndex < centerPoints.length) {
			double heading = Math.toDegrees(centerPoints[curIndex].heading);
			if (heading > 180) {
				heading -= 360;
			}
			else if (heading < -180) {
				heading += 360;
			}
			return heading;
		}
		return null;
	}
	
	public void incrementCounter() {
		curIndex++;
	}
	
	public void resetCounter() {
		curIndex =0;
	}
	
	public Segment[] getCenterPoints() {
		return centerPoints;
	}
	
	public Segment[] getLeftPoints() {
		return leftPoints;
	}
	
	public Segment[] getRightPoints() {
		return rightPoints;
	}
	
    public static void main(String[] args) {
        Waypoint[] points = new Waypoint[] {
                new Waypoint(0, 0, 0),
                new Waypoint(78, 20, Pathfinder.d2r(32))
        };

        PathGenerator path = new PathGenerator(points, 0.01, 120, 200.0, 700.0);
        Segment[] centerSegments = path.getCenterPoints();
        Segment[] leftSegments = path.getLeftPoints();
        Segment[] rightSegments = path.getRightPoints();
       
        double[] heading = new double[centerSegments.length];
        double[] centerX = new double[centerSegments.length];
        double[] centerY = new double[centerSegments.length];
        double[] centerAccel = new double[centerSegments.length];
        double[] centerVelocity = new double[centerSegments.length];
        double[] centerPosition = new double[centerSegments.length];
        double[] leftX = new double[leftSegments.length];
        double[] leftY = new double[leftSegments.length];
        double[] leftAccel = new double[leftSegments.length];
        double[] leftVelocity = new double[leftSegments.length];
        double[] leftPosition = new double[leftSegments.length];
        double[] rightX = new double[rightSegments.length];
        double[] rightY = new double[rightSegments.length];
        double[] rightAccel = new double[rightSegments.length];
        double[] rightVelocity = new double[rightSegments.length];
        double[] rightPosition = new double[rightSegments.length];
        
        path.resetCounter();
        for (int i = 0; i < centerSegments.length; i++) {
        	heading[i] = path.getHeading();
        	centerX[i] = path.getCenterPoint().x;
        	centerY[i] = path.getCenterPoint().y;
        	centerAccel[i] = path.getCenterPoint().acceleration;
        	centerVelocity[i] = path.getCenterPoint().velocity;
        	centerPosition[i] = path.getCenterPoint().position;
        	leftX[i] = path.getLeftPoint().x;
        	leftY[i] = path.getLeftPoint().y;
        	leftAccel[i] = path.getLeftPoint().acceleration;
        	leftVelocity[i] = path.getLeftPoint().velocity;
        	leftPosition[i] = path.getLeftPoint().position;
        	rightX[i] = path.getRightPoint().x;
        	rightY[i] = path.getRightPoint().y;
        	rightAccel[i] = path.getRightPoint().acceleration;
        	rightVelocity[i] = path.getRightPoint().velocity;
        	rightPosition[i] = path.getRightPoint().position;
        	path.incrementCounter();
        }
        
		FalconLinePlot fig4 = new FalconLinePlot(centerX);
		fig4.yGridOn();
		fig4.xGridOn();
		fig4.setYLabel("X (in)");
		fig4.setXLabel("time");
		fig4.setTitle("Position Profile X");
		fig4.addData(rightX, Color.magenta);
		fig4.addData(leftX, Color.blue);

		FalconLinePlot fig0 = new FalconLinePlot(centerY);
		fig0.yGridOn();
		fig0.xGridOn();
		fig0.setYLabel("Y (in)");
		fig0.setXLabel("time");
		fig0.setTitle("Position Profile Y");
		fig0.addData(rightY, Color.magenta);
		fig0.addData(leftY, Color.blue);

		FalconLinePlot fig1 = new FalconLinePlot(centerPosition);
		fig1.yGridOn();
		fig1.xGridOn();
		fig1.setYLabel("Position (in)");
		fig1.setXLabel("time (seconds)");
		fig1.setTitle("Position Profile for Left and Right Wheels \n Left = Blue, Right = Magenta");
		fig1.addData(rightPosition, Color.magenta);
		fig1.addData(leftPosition, Color.blue);

		FalconLinePlot fig2 = new FalconLinePlot(centerVelocity, Color.green, Color.green);
		fig2.yGridOn();
		fig2.xGridOn();
		fig2.setYLabel("Velocity (in/sec)");
		fig2.setXLabel("time (seconds)");
		fig2.setTitle("Velocity Profile for Left and Right Wheels \n Left = Blue, Right = Magenta");
		fig2.addData(rightVelocity, Color.magenta);
		fig2.addData(leftVelocity, Color.blue);

		FalconLinePlot fig3 = new FalconLinePlot(centerAccel, Color.green, Color.green);
		fig3.yGridOn();
		fig3.xGridOn();
		fig3.setYLabel("Accel (in/sec^2)");
		fig3.setXLabel("time (seconds)");
		fig3.setTitle("Accel Profile for Left and Right Wheels \n Left = Blue, Right = Magenta");
		fig3.addData(rightAccel, Color.magenta);
		fig3.addData(leftAccel, Color.blue);

		FalconLinePlot fig5 = new FalconLinePlot(heading);
		fig5.yGridOn();
		fig5.xGridOn();
		fig5.setYLabel("Heading");
		fig5.setXLabel("time");
		fig5.setTitle("Heading Profile");

		FalconLinePlot fig6 = new FalconLinePlot(centerX, centerY);
		fig6.yGridOn();
		fig6.xGridOn();
		fig6.setYLabel("Y");
		fig6.setXLabel("X");
		fig6.setTitle("XY Profile");
  }

}

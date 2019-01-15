package org.usfirst.frc4388.utility;

public class MotionProfileBoxCar
{
	public static double DEFAULT_T1 = 200;	// millisecond
	public static double DEFAULT_T2 = 100; // millisecond
	
	private double startDistance;  // any distance unit
	private double targetDistance;  // any distance unit
	private double maxVelocity;		// velocity unit consistent with targetDistance
	
	// Accel profile
	//
	//  0 T2   T1
	//  | |    |
	//     _____
	//    /     \
	//   /       \___
	//               \      /
	//                \____/   
			
	private double itp;	  			// time between points millisecond
	private double t1 = DEFAULT_T1; // time when accel starts back to 0.  millisecond  (typically use t1 = 2 * t2)
	private double t2 = DEFAULT_T2; // time when accel = max accel.  millisecond  
	
	private double t4;
	private int numFilter1Boxes;
	private int numFilter2Boxes;
	private int numPoints;
	
	private int numITP;
	private double filter1;
	private double filter2;
	private double previousPosition;
	private double previousVelocity;
	private double deltaFilter1;
	private double[] filter2Window;
	private int windowIndex;
	private int pointIndex;

	public MotionProfileBoxCar(double startDistance, double targetDistance, double maxVelocity, double itp) {
		this(startDistance, targetDistance, maxVelocity, itp, DEFAULT_T1, DEFAULT_T2);
	} 
	
	public MotionProfileBoxCar(double startDistance, double targetDistance, double maxVelocity, double itp, double t1, double t2) {
		this.startDistance = startDistance;
		this.targetDistance = targetDistance;
		this.maxVelocity = maxVelocity;
		this.itp = itp;
		this.t1 = t1;
		this.t2 = t2;
		
		initializeProfile();
	} 
	
	private void initializeProfile() {
		// t4 is the time in ms it takes to get to the end point when at max velocity
		t4 = Math.abs((targetDistance - startDistance)/maxVelocity) * 1000;
		
		// We need to make t4 an even multiple of itp
		t4 = (int)(itp * Math.ceil(t4/itp));
		
		// In the case where t4 is less than the accel times, we need to adjust the
		// accel times down so the filters works out.  Lots of ways to do this but
		// to keep things simple we will make t4 slightly larger than the sum of 
		// the accel times.
		if (t4 < t1 + t2) {
			double total = t1 + t2 + t4;
			double t1t2Ratio = t1/t2;
			double t2Adjusted = Math.floor(total / 2 / (1 + t1t2Ratio) / itp);
			if (t2Adjusted % 2 != 0) {
				t2Adjusted -= 1;
			}
			t2 = t2Adjusted * itp;
			t1 = t2 * t1t2Ratio;
			t4 = total - t1 - t2;
		}
		
		// Adjust max velocity so that the end point works out to the correct position.
		maxVelocity = Math.abs((targetDistance - startDistance) / t4) * 1000;

		numFilter1Boxes = (int)Math.ceil(t1/itp);
		numFilter2Boxes = (int)Math.ceil(t2/itp);
		numPoints = (int)Math.ceil(t4/itp);

		numITP = numPoints + numFilter1Boxes + numFilter2Boxes;
		filter1 = 0;
		filter2 = 0;
		previousVelocity = 0;
		previousPosition = startDistance;
		deltaFilter1 = 1.0/numFilter1Boxes;
		filter2Window = new double[numFilter2Boxes];
		windowIndex = 0;
		pointIndex = 0;
		if (startDistance > targetDistance && maxVelocity > 0) {
			maxVelocity = -maxVelocity;
		}
	}
	
	public MotionProfilePoint getNextPoint(MotionProfilePoint point) {
		if (point == null) {
			point = new MotionProfilePoint();
		}
		
		if (pointIndex == 0) {
			point.initialize(startDistance);
			pointIndex++;
			return point;
		}
		else if (pointIndex > numITP) {
			return null;
		}
		
		int input = (pointIndex - 1) < numPoints ? 1 : 0;		
		if (input > 0) {
			filter1 = Math.min(1, filter1 + deltaFilter1);
		}
		else {
			filter1 = Math.max(0, filter1 - deltaFilter1);				
		}
		
		double firstFilter1InWindow = filter2Window[windowIndex];
		if (pointIndex <= numFilter2Boxes) {
			firstFilter1InWindow = 0;
		}
		filter2Window[windowIndex] = filter1;
		
		filter2 += (filter1 - firstFilter1InWindow) / numFilter2Boxes;
		
		point.time = pointIndex * itp / 1000.0;
		point.velocity = filter2 * maxVelocity;
		point.position = previousPosition + (point.velocity + previousVelocity) /  2 * itp / 1000;
		point.acceleration = (point.velocity - previousVelocity) / itp * 1000;
					
		previousVelocity = point.velocity;
		previousPosition = point.position;
		windowIndex++;
		if (windowIndex == numFilter2Boxes) {
			windowIndex = 0;
		}	
		
		pointIndex++;
		
		return point;
	}

	public double getStartDistance() {
		return startDistance;
	}

	public double getTargetDistance() {
		return targetDistance;
	}

	public double getMaxVelocity() {
		return maxVelocity;
	}

	public double getItp() {
		return itp;
	}

	public double getT1() {
		return t1;
	}

	public double getT2() {
		return t2;
	}

	public static void main(String[] args) {
		long startTime = System.nanoTime();
		
		MotionProfileBoxCar mp = new MotionProfileBoxCar(0, 96, 120, 10, 600, 300);
		System.out.println("Time, Position, Velocity, Acceleration");
		MotionProfilePoint point = new MotionProfilePoint();
		while(mp.getNextPoint(point) != null) {
			System.out.println(point.time + ", " + point.position + ", " + point.velocity + ", " + point.acceleration);
		}
		
		long deltaTime = System.nanoTime() - startTime;
		System.out.println("Time Box Car = " + (double)deltaTime * 1E-6 + " ms");
	}
}
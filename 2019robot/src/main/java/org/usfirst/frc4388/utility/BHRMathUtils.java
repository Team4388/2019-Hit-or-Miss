package org.usfirst.frc4388.utility;

public class BHRMathUtils {

	public static double normalizeAngle0To360(double currentAccumAngle) {
		// reduce the angle  
		double normalizedAngle = currentAccumAngle % 360; 

		// force it to be the positive remainder, so that 0 <= angle < 360  
		normalizedAngle = (normalizedAngle + 360) % 360;  
		
		return normalizedAngle;
	}

	public static double adjustAccumAngleToDesired(double currentAccumAngle, double desiredAngle0To360) {
		double normalizedAngle = normalizeAngle0To360(currentAccumAngle); 

		if ( Math.abs(normalizedAngle - desiredAngle0To360) <= 180) {
			double deltaAngle = normalizedAngle - desiredAngle0To360;
			return currentAccumAngle - deltaAngle;
		}
		else {
			double deltaAngle = desiredAngle0To360 - normalizedAngle;
			if (deltaAngle < 0) {
				deltaAngle += 360;
			}
			else {
				deltaAngle -= 360;
			}
			return currentAccumAngle + deltaAngle;
		}
	}

	public static double adjustAccumAngleToClosest180(double currentAccumAngle) {
		double normalizedAngle = Math.abs(normalizeAngle0To360(currentAccumAngle)); 

		if (normalizedAngle < 90 || normalizedAngle > 270) {
			return adjustAccumAngleToDesired(currentAccumAngle, 0);
		}
		else {
			return adjustAccumAngleToDesired(currentAccumAngle, 180);
		}
	}
		
	public static void main(String[] args) {
		System.out.println("Accum angle to desired 721 to 45 = " + adjustAccumAngleToDesired(721,  45));
	}
}

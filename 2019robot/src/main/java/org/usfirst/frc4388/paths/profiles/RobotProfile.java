package org.usfirst.frc4388.paths.profiles;

/**
 * Interface that holds all the corrective values for how each robot actually drives.
 * 
 * @see PathAdapter
 */
public interface RobotProfile {

    // red
    public double getRedBoilerGearXCorrection();

    public double getRedBoilerGearYCorrection();

    public double getRedHopperXOffset();

    public double getRedHopperYOffset();

    // blue
    public double getBlueBoilerGearXCorrection();

    public double getBlueBoilerGearYCorrection();

    public double getBlueHopperXOffset();

    public double getBlueHopperYOffset();

}

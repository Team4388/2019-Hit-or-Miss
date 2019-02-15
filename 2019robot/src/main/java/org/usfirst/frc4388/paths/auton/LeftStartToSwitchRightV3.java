package org.usfirst.frc4388.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc4388.paths.PathBuilder;
import org.usfirst.frc4388.paths.PathBuilder.Waypoint;
import org.usfirst.frc4388.paths.PathContainer;
import org.usfirst.frc4388.utility.control.Path;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.math.Rotation2d;
import org.usfirst.frc4388.utility.math.Translation2d;


public class LeftStartToSwitchRightV3 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(65,270,0,80));
        sWaypoints.add(new Waypoint(65,134,40,80,  "raiseElevator"));
        sWaypoints.add(new Waypoint(110,134,0,80));
        sWaypoints.add(new Waypoint(126,134,0,80));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(65, 270), Rotation2d.fromDegrees(-90.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}
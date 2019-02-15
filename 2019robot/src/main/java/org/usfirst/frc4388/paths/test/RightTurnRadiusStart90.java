package org.usfirst.frc4388.paths.test;

import java.util.ArrayList;

import org.usfirst.frc4388.paths.PathBuilder;
import org.usfirst.frc4388.paths.PathBuilder.Waypoint;
import org.usfirst.frc4388.paths.PathContainer;
import org.usfirst.frc4388.utility.control.Path;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.math.Rotation2d;
import org.usfirst.frc4388.utility.math.Translation2d;

public class RightTurnRadiusStart90 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(60,100,0,0));
        sWaypoints.add(new Waypoint(60,60,30,20));
        sWaypoints.add(new Waypoint(20,60,0,20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(60, 100), Rotation2d.fromDegrees(-90.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}
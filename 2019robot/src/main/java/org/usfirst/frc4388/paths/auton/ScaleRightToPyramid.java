package org.usfirst.frc4388.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc4388.paths.PathBuilder;
import org.usfirst.frc4388.paths.PathBuilder.Waypoint;
import org.usfirst.frc4388.paths.PathContainer;
import org.usfirst.frc4388.utility.control.Path;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.math.Rotation2d;
import org.usfirst.frc4388.utility.math.Translation2d;


public class ScaleRightToPyramid implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(282,80,0,130));
        sWaypoints.add(new Waypoint(230,65,30,130,  "shiftHi"));
        sWaypoints.add(new Waypoint(105,62,0,130,      "shiftLo"));
        sWaypoints.add(new Waypoint(60,62,40,60));
        sWaypoints.add(new Waypoint(60,146,20,60,      "intakeOn"));
        sWaypoints.add(new Waypoint(80,160,0,60));
        sWaypoints.add(new Waypoint(150,160,0,40));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(282, 80), Rotation2d.fromDegrees(180)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}
package org.usfirst.frc4388.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc4388.paths.PathBuilder;
import org.usfirst.frc4388.paths.PathBuilder.Waypoint;
import org.usfirst.frc4388.paths.PathContainer;
import org.usfirst.frc4388.utility.control.Path;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.math.Rotation2d;
import org.usfirst.frc4388.utility.math.Translation2d;


public class LeftStartToScaleRightAPRV2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,274,0,40));
        sWaypoints.add(new Waypoint(55,274,20,40));
        sWaypoints.add(new Waypoint(55,250,0,110,    "shiftHi"));
        sWaypoints.add(new Waypoint(55,210,0,110));
        sWaypoints.add(new Waypoint(55,105,80,110));
        sWaypoints.add(new Waypoint(140,100,0,80));
        sWaypoints.add(new Waypoint(270,115,0,80,  "shiftLow"));
        sWaypoints.add(new Waypoint(300,120,0,80, "raiseElevator"));
        sWaypoints.add(new Waypoint(330,120,20,80));
        sWaypoints.add(new Waypoint(330,100,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(18, 274), Rotation2d.fromDegrees(180)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
}
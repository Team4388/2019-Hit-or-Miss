package org.usfirst.frc4388.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc4388.paths.PathBuilder;
import org.usfirst.frc4388.paths.PathBuilder.Waypoint;
import org.usfirst.frc4388.paths.PathContainer;
import org.usfirst.frc4388.utility.control.Path;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.math.Rotation2d;
import org.usfirst.frc4388.utility.math.Translation2d;


public class RightStartToScaleLeftSaveV2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,51,0,110));
        sWaypoints.add(new Waypoint(60,51,0,130,    "shiftHi"));
        sWaypoints.add(new Waypoint(159,51,0,130));
        sWaypoints.add(new Waypoint(220,51,30,110));
        sWaypoints.add(new Waypoint(218,115,0,60));
        sWaypoints.add(new Waypoint(218,145,0,130));
        sWaypoints.add(new Waypoint(218,175,0,130));
        sWaypoints.add(new Waypoint(218,205,0,130));
        sWaypoints.add(new Waypoint(218,234,25,50));
        sWaypoints.add(new Waypoint(250,234,0,50,     "raiseElevator"));
        sWaypoints.add(new Waypoint(270,231,0,40));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(18, 51), Rotation2d.fromDegrees(180)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
}
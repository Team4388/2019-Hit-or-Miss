package org.usfirst.frc4388.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc4388.paths.PathBuilder;
import org.usfirst.frc4388.paths.PathBuilder.Waypoint;
import org.usfirst.frc4388.paths.PathContainer;
import org.usfirst.frc4388.utility.control.Path;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.math.Rotation2d;
import org.usfirst.frc4388.utility.math.Translation2d;


public class LeftStartToScaleRightSave implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,274,0,110));
        sWaypoints.add(new Waypoint(159,274,0,110));
        sWaypoints.add(new Waypoint(220,274,40,110));
        sWaypoints.add(new Waypoint(214,210,0,60));
        sWaypoints.add(new Waypoint(210,180,0,100));
        sWaypoints.add(new Waypoint(209,150,0,80));
        sWaypoints.add(new Waypoint(209,120,0,70));
        sWaypoints.add(new Waypoint(214,87,30,50));
        sWaypoints.add(new Waypoint(250,87,0,50,   "raiseElevator"));
        sWaypoints.add(new Waypoint(270,90,0,40));

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
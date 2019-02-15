package org.usfirst.frc4388.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc4388.paths.PathBuilder;
import org.usfirst.frc4388.paths.PathBuilder.Waypoint;
import org.usfirst.frc4388.paths.PathContainer;
import org.usfirst.frc4388.utility.control.Path;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.math.Rotation2d;
import org.usfirst.frc4388.utility.math.Translation2d;


public class RightStartToScaleLeftAPRV2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,51,0,40));
        sWaypoints.add(new Waypoint(55,51,20,40));
        sWaypoints.add(new Waypoint(55,75,0,110,      "shiftHi"));
        sWaypoints.add(new Waypoint(55,130,0,110));
        sWaypoints.add(new Waypoint(55,230,80,110));
        sWaypoints.add(new Waypoint(140,235,0,80));
        sWaypoints.add(new Waypoint(240,220,0,80,    "shiftLow"));
        sWaypoints.add(new Waypoint(270,215,0,80,   "raiseElevator"));
        sWaypoints.add(new Waypoint(310,215,20,80));
        sWaypoints.add(new Waypoint(310,235,0,60));

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
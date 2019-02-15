package org.usfirst.frc4388.paths.test;

import java.util.ArrayList;

import org.usfirst.frc4388.paths.PathBuilder;
import org.usfirst.frc4388.paths.PathBuilder.Waypoint;
import org.usfirst.frc4388.paths.PathContainer;
import org.usfirst.frc4388.utility.control.Path;
import org.usfirst.frc4388.utility.math.RigidTransform2d;
import org.usfirst.frc4388.utility.math.Rotation2d;
import org.usfirst.frc4388.utility.math.Translation2d;

public class SCurveReversed implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint( 20,  0,  0,  0));
        sWaypoints.add(new Waypoint( 60,  0, 20, 20));
        sWaypoints.add(new Waypoint(100, 15, 20, 20));
        sWaypoints.add(new Waypoint(130, 15,  0, 20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 0), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":60},"speed":0,"radius":0,"comment":""},{"position":{"x":60,"y":60},"speed":20,"radius":20,"comment":""},{"position":{"x":100,"y":75},"speed":20,"radius":20,"comment":""},{"position":{"x":130,"y":75},"speed":20,"radius":0,"comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: SCurveReversed
}
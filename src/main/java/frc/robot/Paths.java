package frc.robot;

import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Pathfinder;

public class Paths {

    public static Waypoint[] fwdThenLeft = new Waypoint[]{
        new Waypoint(0.0, 4.0, Pathfinder.r2d(0.0)),
        new Waypoint(3.0, 4.0, Pathfinder.r2d(0.0)),
        new Waypoint(10.0, 4.0, Pathfinder.r2d(0.0)),
        new Waypoint(12.0, 8.0, Pathfinder.r2d(1.0)),
        new Waypoint(15.0, 10.5, Pathfinder.r2d(0.0)),
    };

}
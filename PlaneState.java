package airplane.g5;
import java.util.ArrayList;
import java.util.Set;

import airplane.g5.Trajectory;
import airplane.sim.Plane;

import java.awt.geom.Point2D;
import java.awt.geom.Line2D;

public class PlaneState
{
 Plane plane;
	  
  public PlaneState(Plane p)
  {
    plane = p;
  }
  

  // copy constructor
  public PlaneState(PlaneState planeState)
  {
      this(planeState.plane);
      this.state = planeState.state;
      this.bearingOrig = planeState.bearingOrig;
      this.landingLock = planeState.landingLock;
      if (planeState.currentTarget != null)
        this.currentTarget = new Point2D.Double(planeState.currentTarget.getX(), planeState.currentTarget.getY());
      if (planeState.currentZone != null)
        this.currentZone = new Zone(planeState.currentZone);
      if (planeState.simulationTrajectory != null)
        this.simulationTrajectory = new Trajectory(planeState.simulationTrajectory);
      landed = planeState.landed;
      pathIter = planeState.pathIter;
      orbitDirection = planeState.orbitDirection;
      zoneRadius = planeState.zoneRadius;
      routeDirection = planeState.routeDirection;
      takeoffAngle = planeState.takeoffAngle;
      // do not deep copy route
      route = planeState.route;
      // do not deep copy path
      path = planeState.path;
      // do not deep copy walls
      walls = planeState.walls;
  }
  


  // states
  public enum States
  {
    NULL_STATE, ORBIT_STATE, SPIRAL_STATE, COLLISION_STATE;
  }

  States state = States.NULL_STATE;

  double bearingOrig = 0;
  boolean landingLock = false;
  boolean landed = false;
  Zone currentZone;
  Point2D currentTarget;
  Trajectory simulationTrajectory;
  Route route;
  ArrayList<Waypoint> path;
  int pathIter = 1; // source waypoint located at 0
  Set<Line2D> walls;
  int orbitDirection = 1;
  double zoneRadius = AStarPlayer.WAYPOINT_ZONE_RADIUS;
  int routeDirection = Route.FORWARD;
  double takeoffAngle = -1;
  
}

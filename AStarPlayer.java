package airplane.g5;

import java.util.List;
import java.awt.geom.Point2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import org.apache.log4j.Logger;

import airplane.sim.Plane;
import airplane.sim.SimulationResult;

public class AStarPlayer  extends airplane.sim.Player
{

  private Logger logger = Logger.getLogger(this.getClass()); // for logging

  // Map from plane object to plane state object
  private List<PlaneState> planeStateMap;
  private List<PlaneState> planeStateMapSim;

  private HashSet<Integer> departedPlanes = new HashSet<Integer>();
  private ArrayList<Plane> flyingPlanes = new ArrayList<Plane>();
  private Set<Route> routeSet = new HashSet<Route>();

  private static final double WAITING = -1;
  private static final double LANDED = -2;
  private static final double MAX_TURN = 9.99999999999999;
  private static final double CRITICAL_COLLISION_ZONE_ANGLE = 10;
  private static final float COLLISION_ZONE_RADIUS = 5;
  private static final double SAFE_SIM_DIST = 7;
  private static final float AIRPORT_ZONE_RADIUS = 1;
  private static final int CRITICAL_ROUTE_TRAFFIC = 5;
  private static final int CRITICAL_WAYPOINT_TRAFFIC = 10;
  static final double WAYPOINT_ZONE_RADIUS = 7;
  private static final double LANDING_WAYPOINT_ZONE_RADIUS = 11;

  private boolean testDepart = false;

  @Override
  public String getName()
  {
    return "AStar Player";
  }

  private String reasonToString(int reason)
  {
    String reasonString = "";
    switch(reason)
    {
    case SimulationResult.ILLEGAL_BEARING:
      reasonString = "illegal bearing";
      break;
    case SimulationResult.NORMAL:
      reasonString = "normal";
      break;
    case SimulationResult.NULL_BEARINGS:
      reasonString = "null bearings";
      break;
    case SimulationResult.OUT_OF_BOUNDS:
      reasonString = "out of bounds";
      break;
    case SimulationResult.STOPPED:
      reasonString = "stopped";
      break;
    case SimulationResult.TOO_CLOSE:
      reasonString = "too close";
      break;
    case SimulationResult.TOO_EARLY:
      reasonString = "too early";
      break;
    }

    return reasonString;
  }

  private boolean depart(int planeId, int round, ArrayList<Plane> planes)
  {
    boolean depart = true;
    ArrayList<Plane> planesToSim = new ArrayList<Plane>();
    planesToSim.addAll(flyingPlanes);
    planesToSim.add(planes.get(planeId));

    boolean upgradeToCollision = false;
    Point2D collisionAvoidTarget = new Point2D.Double(0, 0);
    
    ArrayList<Waypoint> aStarPath = new ArrayList<Waypoint> ();
    HashSet<Line2D> aStarWalls = new HashSet<Line2D> ();
    double aStarZoneRadius = WAYPOINT_ZONE_RADIUS;
    boolean upgradeToAStar = false;

    // refresh simulator state
    refreshSimState();

    // first try straight line path
    planeStateMapSim.get(planeId).state = PlaneState.States.NULL_STATE;
    SimulationResult result = startSimulation(planesToSim, round);

    if (result.getReason() == SimulationResult.TOO_CLOSE)
    {
      // check collision bearings of colliding planes:
      depart = false;
      ArrayList<Plane> simPlanes = result.getPlanes();
      for (Plane simPlane1: simPlanes)
      {
        Point2D.Double simP1 = simPlane1.getLocation();
        if (simPlane1.id == planeId)
        {
          for (Plane simPlane2: simPlanes)
          {
            Point2D.Double simP2 = simPlane2.getLocation();
            // form collision zone if (180 + d) < angle1 - angle2 < (180 + d)
            if (simPlane1.id != simPlane2.id && simP1.distance(simP2) <= 5)
            {
              double angleOfApproach = addBearings(simPlane1.getBearing(), - simPlane2.getBearing());
              if ((angleOfApproach <= 180 + CRITICAL_COLLISION_ZONE_ANGLE) && (angleOfApproach >= 180 - CRITICAL_COLLISION_ZONE_ANGLE) && simPlane2.getBearing() >= 0)
              {
                PlaneState planeState1 = planeStateMap.get(simPlane1.id);
                PlaneState planeState2 = planeStateMap.get(simPlane2.id);
                Plane plane1 = planeState1.plane;
                Plane plane2 = planeState2.plane;
                Vector approachVector = new Vector(simPlane2.getBearing() - 90);
                Vector avoidVector = approachVector.rotate90AntiClockwise();
                avoidVector.multiply(COLLISION_ZONE_RADIUS);
                Vector avoidVectorOpposite = avoidVector.rotateOpposite();

                float c = (float) plane1.getLocation().distance(plane2.getLocation());
                float x = (c*c - COLLISION_ZONE_RADIUS*COLLISION_ZONE_RADIUS)/(2*c);
                if (x > 0)
                {
                  Vector locationVector = new Vector(plane2.getLocation());
                  approachVector.multiply(c - x);
                  Vector collisionVector = Vector.addVectors(locationVector, approachVector);
                  Vector avoidVectorAbsolute = Vector.addVectors(collisionVector, avoidVector);
                  Vector avoidVectorAbsoluteOpposite = Vector.addVectors(collisionVector, avoidVectorOpposite);

                  // refresh simulator state
                  refreshSimState();
                  planeStateMapSim.get(planeId).currentTarget = avoidVectorAbsolute.getPoint();
                  collisionAvoidTarget = planeStateMapSim.get(planeId).currentTarget;
                  planeStateMapSim.get(planeId).state = PlaneState.States.COLLISION_STATE;

                  if (collisionAvoidTarget.distance(plane1.getDestination()) > SAFE_SIM_DIST)
                  {
                    result = startSimulation(planesToSim, round);
                    if (result.getReason() == SimulationResult.NORMAL)
                    {
                      depart = true;
                      upgradeToCollision = true;
                      break;
                    }
                  }
                  // refresh simulator state
                  refreshSimState();
                  planeStateMapSim.get(planeId).currentTarget = avoidVectorAbsoluteOpposite.getPoint();
                  collisionAvoidTarget = planeStateMapSim.get(planeId).currentTarget;
                  planeStateMapSim.get(planeId).state = PlaneState.States.COLLISION_STATE;

                  if (collisionAvoidTarget.distance(plane1.getDestination()) > SAFE_SIM_DIST)
                  {
                    result = startSimulation(planesToSim, round);
                    if (result.getReason() == SimulationResult.NORMAL)
                    {
                      depart = true;
                      upgradeToCollision = true;
                      break;
                    }
                  }
                }
              }
            }
          }
          break;
        }
      }
    }
    
    if (result.getReason() == SimulationResult.TOO_CLOSE) // A* routing if collision with a flow
    {
      ArrayList<Plane> simPlanes = result.getPlanes();
      for (Plane simPlane1: simPlanes)
      {
        Point2D.Double simP1 = simPlane1.getLocation();
        if (simPlane1.id == planeId)
        {
          PlaneState planeState = planeStateMap.get(planeId);
          for (Plane simPlane2: simPlanes)
          {
            Point2D.Double simP2 = simPlane2.getLocation();
            if (simPlane1.id != simPlane2.id && simP1.distance(simP2) <= 5)
            {
              // traffic more than flow threshold and not this flow
              if (planeStateMap.get(simPlane2.id).route.currentTraffic >= CRITICAL_ROUTE_TRAFFIC && planeStateMap.get(simPlane2.id).route != planeStateMap.get(simPlane1.id).route)
              {
                // get current flow routes
                Set<Route> currentFlowRoutes = new HashSet<Route> ();
                for (Route route : routeSet)
                {
                  if (route.currentTraffic >= CRITICAL_ROUTE_TRAFFIC && route != planeStateMap.get(simPlane1.id).route)
                  {
                    currentFlowRoutes.add(route);
                  }
                }
                // update walls
                walls.clear();
                for (Route flow : currentFlowRoutes)
                {
                  walls.add(new Line2D.Double(flow.waypoint1.point, flow.waypoint2.point));
                }
                // update current waypoints
                waypointSet.clear();
                visibilityMap.clear();
                for (Route flow : currentFlowRoutes)
                {
                  // apply pruning on busy waypoints
                  if (flow.waypoint1.currentTraffic < CRITICAL_WAYPOINT_TRAFFIC)
                    addWaypoint(flow.waypoint1);
                  if (flow.waypoint2.currentTraffic < CRITICAL_WAYPOINT_TRAFFIC)
                    addWaypoint(flow.waypoint2);
                }
                // calculate A* path
                ArrayList<Waypoint> path = AStar(new Waypoint(planeState.plane.getLocation()), new Waypoint(planeState.plane.getDestination()));
                if (path == null) // recomute path without pruning
                {
                  // update current waypoints
                  waypointSet.clear();
                  visibilityMap.clear();
                  for (Route flow : currentFlowRoutes)
                  {
                    addWaypoint(flow.waypoint1);
                    addWaypoint(flow.waypoint2);
                  }
                  // calculate A* path
                  path = AStar(new Waypoint(planeState.plane.getLocation()), new Waypoint(planeState.plane.getDestination()));
                }
                // simulate trajectory
                if (path != null)
                {
                  // refresh simulator state
                  refreshSimState();
                  planeStateMapSim.get(planeId).path = path;
                  planeStateMapSim.get(planeId).state = PlaneState.States.ORBIT_STATE;
                  planeStateMapSim.get(planeId).walls = walls;
                  if (path.size() <= 2) // create landing zone
                  {
                    planeStateMapSim.get(planeId).zoneRadius = LANDING_WAYPOINT_ZONE_RADIUS;
                  }

                  result = startSimulation(planesToSim, round);
                  if (result.getReason() == SimulationResult.NORMAL)
                  {
                    depart = true;
                    upgradeToAStar = true;
                    aStarPath = path;
                    aStarZoneRadius = planeStateMapSim.get(planeId).zoneRadius;

                    // make copy of walls
                    for (Line2D wall : walls)
                    {
                      Line2D newWall = new Line2D.Double(wall.getP1(), wall.getP2());
                      aStarWalls.add(newWall);
                    }
                    // update traffic on waypoints in-between source-dest
                    for (Waypoint waypoint : path)
                    {
                      if (waypoint != path.get(0) && waypoint != path.get(path.size() - 1))
                        waypoint.currentTraffic++;
                    }
                    break;
                  }
                }
              }
            }
          }
        }
      }

    }

    if (!result.isSuccess())
    {
      //logger.info("COULDN'T DEPART BECAUSE " + reasonToString(result.getReason()));
      depart = false;
      /*if (upgradeToAStar && !testDepart)
      {
        testDepart = true;
        depart = true;
      }*/
    }

    if (upgradeToCollision)
    {
      PlaneState planeState = planeStateMap.get(planeId);
      planeState.state = PlaneState.States.COLLISION_STATE;
      planeState.currentTarget = collisionAvoidTarget;
    }
    else if (upgradeToAStar)
    {
      PlaneState planeState = planeStateMap.get(planeId);
      planeState.state = PlaneState.States.ORBIT_STATE;
      planeState.path = aStarPath;
      planeState.walls = aStarWalls;
      planeState.zoneRadius = aStarZoneRadius;
    }
    else
    {
      PlaneState planeState = planeStateMap.get(planeId);
      planeState.state = PlaneState.States.NULL_STATE;
    }
    return depart;
  }

  public void refreshSimState()
  {
    planeStateMapSim.clear();
    for (PlaneState planeState : planeStateMap)
    {
      PlaneState planeStateSim = new PlaneState(planeState);
      planeStateMapSim.add(planeStateSim);
    }
  }

  /*
   * This is called at the beginning of a new simulation.
   * Each Plane object includes its current location (origin), destination, and
   * current bearing, which is -1 to indicate that it's on the ground.
   */
  @Override
  public void startNewGame(ArrayList<Plane> planes)
  {
    logger.info("Starting new game!");

    planeStateMap = new ArrayList<PlaneState> ();
    planeStateMapSim = new ArrayList<PlaneState> ();
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      PlaneState planeState = new PlaneState(plane);
      planeStateMap.add(plane.id, planeState);
    }
    // initialize ids to distinguish planes
    for (int i = 0; i < planes.size(); i++)
    {
      Plane p = planes.get(i);
      p.id = i;
      planes.set(i, p);
    }

    // compute routes
    for (PlaneState planeState : planeStateMap)
    {
      Point2D p1 = planeState.plane.getLocation();
      Point2D p2 = planeState.plane.getDestination();
      // check if this route can be clubbed with an existing route
      Route route = new Route(new Waypoint(p1), new Waypoint(p2));
      for (Route routeMerge : routeSet)
      {
        Point2D wpP1 = routeMerge.waypoint1.point;
        Point2D wpP2 = routeMerge.waypoint2.point;

        if (wpP1.distance(p1) < AIRPORT_ZONE_RADIUS)
        {
          if (wpP2.distance(p2) < AIRPORT_ZONE_RADIUS)
            route = routeMerge;
        }
        if (wpP1.distance(p2) < AIRPORT_ZONE_RADIUS)
        {
          if (wpP2.distance(p1) < AIRPORT_ZONE_RADIUS)
            route = routeMerge;
        }
      }
      routeSet.add(route);
      planeState.route = route;
    }
  }


  private Comparator<Plane> INCREASING_DEPT = new Comparator<Plane>()
  {
    public int compare(Plane p1, Plane p2)
    {
      if (p2.getDepartureTime() < p1.getDepartureTime())
      {
        return 1;
      }
      else if (p1.getDepartureTime() < p2.getDepartureTime())
      {
        return -1;
      }
      else
      {
        return 0;
      }
    }
  };

  private Comparator<Plane> DECREASING_COST = new Comparator<Plane>()
  {
    public int compare(Plane p1, Plane p2)
    {
      double cost1 = p1.getDepartureTime() + p1.getLocation().distance(p1.getDestination());
      double cost2 = p2.getDepartureTime() + p2.getLocation().distance(p2.getDestination());
      if (cost1 < cost2)
      {
        return 1;
      }
      else if (cost2 < cost1)
      {
        return -1;
      }
      else
      {
        return 0;
      }
    }
  };


  private void dispatchNextPlane()
  {
//	if (departureIds.size() > 1) {
//		departureIds.remove(0);
//		flyingPlane = departureIds.get(0);
//	}
  }

  /*
   * This is used when you're running your own simulation
   */
  @Override
  protected double[] simulateUpdate(ArrayList<Plane> planes, int round, double[] bearings)
  {
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      if (planeStateMapSim.get(plane.id).plane != plane)
      {
        planeStateMapSim.get(plane.id).plane = plane;
      }
    }
    for (int i = 0; i < planes.size(); i++)
    {
      Plane p = planes.get(i);
      PlaneState planeState = planeStateMapSim.get(p.id);
      if (p.getBearing() != -1 && p.getBearing() != -2)
      {
        if (planeState.state == PlaneState.States.COLLISION_STATE)
        {
          // check if reached: change to spiral state
          if (reachedCollisionZone(planeState))
            planeState.state = PlaneState.States.SPIRAL_STATE;
        }
        else if (planeState.state == PlaneState.States.SPIRAL_STATE)
        {
          // move in spiral pattern towards target
          bearings[i] = spiralOrbit(planeState, p.getDestination());
        }
        else if (planeState.state == PlaneState.States.ORBIT_STATE)
        {
          // check if next waypoint visible
          Point2D wpPoint = planeState.path.get(planeState.pathIter).point;
          double deltaBearing = 0;
          if (planeState.path.size() > 2)
          {
            Point2D wpPointNext = planeState.path.get(planeState.pathIter + 1).point;
            double wpPointNextBearing = calculateBearing(p.getLocation(), (Point2D.Double) wpPointNext);
            deltaBearing = addBearings(wpPointNextBearing, -p.getBearing());
          }
          if ((deltaBearing <= 10 || deltaBearing >= 350) && p.getLocation().distance(wpPoint) <= (planeState.zoneRadius + .1))
          {
            planeState.pathIter++;
            if (planeState.pathIter >= planeState.path.size() - 1) // last waypoint
            {
              bearings[i] = spiralOrbit(planeState, p.getDestination());
            }
            else // join next waypoint
            {
              wpPoint = planeState.path.get(planeState.pathIter).point;
              bearings[i] = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, planeState.orbitDirection);
            }
          }
          else
          {
            // Move in orbital tangent towards waypoint zone
            bearings[i] = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, planeState.orbitDirection);
          }
        }
      }

      else
      {
        if (p.getBearing() != -2)
        {
          if (planeState.state == PlaneState.States.NULL_STATE)
          {
            bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
          }
          else if (planeState.state == PlaneState.States.COLLISION_STATE)
          {
            // Move in collision orbital tangent
            bearings[i] = collisionOrbit(planeState, (Point2D.Double) planeState.currentTarget);
          }
          else if (planeState.state == PlaneState.States.ORBIT_STATE)
          {
            // Move in orbital tangent towards first waypoint
            Point2D wpPoint = planeState.path.get(planeState.pathIter).point;
            double bearing = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, 1);
            if(checkFlowIntersection(planeState, wpPoint, bearing))
            {
              bearing = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, -1);
              planeState.orbitDirection = -1;
            }
            bearings[i] = bearing;
          }
        }
      }
    }
    return bearings;
  }

  /*
   * This is called at each step of the simulation.
   * The List of Planes represents their current location, destination, and current
   * bearing; the bearings array just puts these all into one spot.
   * This method should return an updated array of bearings.
   */
  @Override
  public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearings)
  {
    // make copy of PlaneState
    refreshSimState();
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      planeStateMap.set(planeStateMapSim.get(plane.id).plane.id, planeStateMapSim.get(plane.id));
    }
    for (int i = 0; i < planes.size(); i++)
    {
      Plane p = planes.get(i);
      PlaneState planeState = planeStateMap.get(p.id);
      if (departedPlanes.contains(p.id) && p.getBearing() != -2)
      {
        if (planeState.state == PlaneState.States.NULL_STATE)
        {
        	planeState.state = PlaneState.States.NULL_STATE;
        }
        if (planeState.state == PlaneState.States.COLLISION_STATE)
        {
          // check if reached: change to spiral state
          if (reachedCollisionZone(planeState))
            planeState.state = PlaneState.States.SPIRAL_STATE;
        }
        else if (planeState.state == PlaneState.States.SPIRAL_STATE)
        {
          // move in spiral pattern towards target
          bearings[i] = spiralOrbit(planeState, p.getDestination());
        }
        else if (planeState.state == PlaneState.States.ORBIT_STATE)
        {
          // check if next waypoint visible
          Point2D wpPoint = planeState.path.get(planeState.pathIter).point;
          double deltaBearing = 0;
          if (planeState.path.size() > 2)
          {
            Point2D wpPointNext = planeState.path.get(planeState.pathIter + 1).point;
            double wpPointNextBearing = calculateBearing(p.getLocation(), (Point2D.Double) wpPointNext);
            deltaBearing = addBearings(wpPointNextBearing, -p.getBearing());
          }
          if ((deltaBearing <= 10 || deltaBearing >= 350) && p.getLocation().distance(wpPoint) <= (planeState.zoneRadius + .1))
          {
            planeState.pathIter++;
            if (planeState.pathIter >= planeState.path.size() - 1) // last waypoint
            {
              bearings[i] = spiralOrbit(planeState, p.getDestination());
            }
            else // join next waypoint
            {
              wpPoint = planeState.path.get(planeState.pathIter).point;
              bearings[i] = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, planeState.orbitDirection);
            }
          }
          else
          {
            // Move in orbital tangent towards waypoint zone
            bearings[i] = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, planeState.orbitDirection);
          }
        }
      }

      else if (round >= p.getDepartureTime() && p.getBearing() != -2/* && !testDepart*/)
      {
        if (depart(p.id, round, planes) && p.getBearing() != -2)
        {
          if (planeState.state == PlaneState.States.NULL_STATE)
          {
            bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
          }
          else if (planeState.state == PlaneState.States.COLLISION_STATE)
          {
            // Move in orbital tangent
            bearings[i] = collisionOrbit(planeState, (Point2D.Double) planeState.currentTarget);
          }
          else if (planeState.state == PlaneState.States.ORBIT_STATE)
          {
            // Move in orbital tangent towards first waypoint
            Point2D wpPoint = planeState.path.get(planeState.pathIter).point;
            double bearing = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, 1);
            if(checkFlowIntersection(planeState, wpPoint, bearing))
            {
              bearing = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, -1);
              planeState.orbitDirection = -1;
            }
            bearings[i] = bearing;
          }
          departedPlanes.add(i);
          planeState.route.currentTraffic++;
          if (!flyingPlanes.contains(p))
          {
            flyingPlanes.add(p);
          }
        }
      }
      else if (p.getBearing() == -2)
      {
        if (!planeState.landed)
        {
          planeState.landed = true;
          planeState.route.currentTraffic--;
          if (planeState.path != null)
          {
            for (Waypoint waypoint : planeState.path)
            {
              if (waypoint != planeState.path.get(0) && waypoint != planeState.path.get(planeState.path.size() - 1))
                waypoint.currentTraffic--;
            }
          }
        }
        flyingPlanes.remove(p);
      }
    }

    return bearings;
  }

  public boolean checkFlowIntersection(PlaneState planeState, Point2D center, double degree)
  {
    // check intersection with flow walls
    Plane plane = planeState.plane;
    boolean intersects = false;
    Vector locationVector = new Vector(plane.getLocation());
    Vector directionVector = new Vector(degree - 90);
    directionVector.multiply((float) plane.getLocation().distance(center));
    Vector pathVector = Vector.addVectors(locationVector, directionVector);
    Line2D pathLine = new Line2D.Double(planeState.plane.getLocation(), pathVector.getPoint());
    for (Line2D wall : planeState.walls)
    {
      if(wall.intersectsLine(pathLine))
        intersects = true;
    }
    return intersects;
  }

  public boolean reachedCollisionZone(PlaneState planeState)
  {
    Plane plane = planeState.plane;
    if(plane.getLocation().distance(planeState.currentTarget) < 1)
      return true;
    else
      return false;
  }

  public double joinOrbit(PlaneState planeState, Point2D.Double center, double radius, int direction)
  {
    planeState.state = PlaneState.States.ORBIT_STATE;
    double bearing = getOrbitBearing(planeState, center, radius, direction);
    return turn(planeState, bearing);
  }

  public double spiralOrbit(PlaneState planeState, Point2D.Double destination)
  {
    planeState.state = PlaneState.States.SPIRAL_STATE;
    return reachTarget(planeState, destination);
  }

  public double collisionOrbit(PlaneState planeState, Point2D.Double destination)
  {
    planeState.state = PlaneState.States.COLLISION_STATE;
    return reachTarget(planeState, destination);
  }

  public double reachTarget(PlaneState planeState, Point2D.Double destination)
  {
    Plane plane = planeState.plane;
    double bearingDest = calculateBearing(plane.getLocation(), destination);
    return turn(planeState, bearingDest);

  }

  public double turn(PlaneState planeState, double bearingDest)
  {
    if (planeState.plane.getBearing() == WAITING)
      return bearingDest;
    double bearingOrig = planeState.plane.getBearing();
    double bearingDelta = addBearings(bearingDest, -bearingOrig);
    double bearing = -1;
    if (bearingDelta >= 350 || bearingDelta <= 10)
    {
      bearing = bearingDest;
    }
    else if (bearingDelta > 10 && bearingDelta <= 180)
    {
      bearing = addBearings(bearingOrig, MAX_TURN);
    }
    else if (bearingDelta < 350 && bearingDelta > 180)
    {
      bearing = addBearings(bearingOrig, -MAX_TURN);
    }
    return bearing;
  }

  public double getOrbitBearing(PlaneState planeState, Point2D center, double radius, int direction)
  {
    Plane plane = planeState.plane;
    double dist = plane.getLocation().distance(center);
    double sin = radius / dist;
    if (sin > 1) // inside the circle
      return -1;
    double radians = Math.asin(sin);
    double relativeDegree = Math.toDegrees(radians);
    double referenceDegree = calculateBearing(plane.getLocation(), (Point2D.Double)center);
    double degree = addBearings(referenceDegree, direction * relativeDegree);
    return degree;
  }

  public double addBearings(double bearingOrig, double bearingDelta)
  {
    double retval = (bearingOrig + bearingDelta) % 360;
    if (retval < 0)
    {
      retval = 360 + retval;
    }
    if (retval >= 360)
    {
      retval = retval % 360;
    }
    return retval;
  }


  // A* specific
  private Set<Waypoint> waypointSet = new HashSet<Waypoint> ();
  private Map<Waypoint, Set<Waypoint> > visibilityMap = new HashMap<Waypoint, Set<Waypoint> > ();
  private Set<Line2D> walls = new HashSet<Line2D> ();

  private static final int maxX = 100, maxY = 100;

  void addWaypoint(Waypoint waypoint)
  {
    addToVisibilityMap(waypoint);
    waypointSet.add(waypoint);
  }

  protected boolean isInLineOfSight(double x, double y, double newX, double newY)
  {
    Line2D.Double pathLine = new Line2D.Double(x,y,newX,newY);
    for(Line2D l : walls)
    {
      // account for on the edge points as visible
      if (pathLine.getP1().distance(l.getP1()) > .1 && 
          pathLine.getP2().distance(l.getP2()) > .1 &&
          pathLine.getP1().distance(l.getP2()) > .1 &&
          pathLine.getP2().distance(l.getP1()) > .1)
      {
        if(l.intersectsLine(pathLine))
          return false;
      }
    }
    return true;
  }

  protected boolean isInLineOfSight(Point2D point1, Point2D point2)
  {
    return isInLineOfSight(point1.getX(), point1.getY(), point2.getX(), point2.getY());
  }

  private double GetAStarDistance (Waypoint source, Waypoint target)
  {
    if (isInLineOfSight(source.point, target.point))
    {
      return source.point.distance(target.point);
    }
    ArrayList<Waypoint> path = AStar (source, target);
    if (path != null)
    {
      Waypoint waypointLast = path.get(path.size() - 1);
      return waypointLast.storedSourceDistance;
    }
    else
    {
      return 1.5*maxX;
    }
  }

  // A* path finding algo
  private ArrayList<Waypoint> AStar(Waypoint waypointSource, Waypoint waypointTarget)
  {
    // add target to visibilityMap and waypointSet
    addWaypoint(waypointTarget);
    // add source to visibilityMap and waypointSet
    addWaypoint(waypointSource);

    // Mark source waypoint as openList and calculate F score
    waypointSource.markAsOpenList();
    double sourceDistance = 0;
    waypointSource.updateFScore(sourceDistance, waypointTarget);

    Waypoint waypointCurrent = waypointSource;

    while (true)
    {
      // Check if unreachable
      if (waypointCurrent == null)
      {
        cleanupWaypoints(waypointSet);
        return null;
      }
      // Move towards waypointTarget
      Set<Waypoint> visibleWaypoints = visibilityMap.get(waypointCurrent);
      for (Waypoint waypoint : visibleWaypoints)
      {
        if (!waypoint.closedList && /*fail safe condition*/ waypoint != waypointCurrent)
        {
          double newDistance = waypointCurrent.currentSourceDistance + Waypoint.getDistance(waypointCurrent, waypoint);
          if (newDistance < waypoint.currentSourceDistance || waypoint.currentSourceDistance == -1)
          {
            waypoint.markAsOpenList();
            waypoint.setParent(waypointCurrent);
            waypoint.updateFScore(newDistance, waypointTarget);
            // Check for exit condition
            if (waypoint == waypointTarget)
            {
              ArrayList<Waypoint> path = getPath(waypointSet, waypointSource, waypointTarget);
              cleanupWaypoints(waypointSet);
              return path;
            }
          }
        }
      }
      waypointCurrent = leastFScore(waypointSet, waypointCurrent);
      //log.trace("waypointCurrent: " + waypointCurrent + " waypointSource: " + waypointSource + " waypointTarget: " + waypointTarget);
      if (waypointCurrent != null)
        waypointCurrent.markAsClosedList();
    }
  }

  private void addToVisibilityMap(Waypoint waypoint1)
  {
    Set<Waypoint> waypointSet1 = new HashSet<Waypoint>();
    visibilityMap.put(waypoint1, waypointSet1);
    for (Waypoint waypoint2 : waypointSet)
    {
      if(isInLineOfSight(waypoint1.point, waypoint2.point))
      {
        Set<Waypoint> waypointSet2 = visibilityMap.get(waypoint2);
        waypointSet1.add(waypoint2);
        waypointSet2.add(waypoint1);
      }
    }
  }

  private Waypoint leastFScore(Set<Waypoint> waypoints, Waypoint waypointCurrent)
  {
    Waypoint leastScoreWaypoint = null;
    double leastScore = -1;
    for (Waypoint waypoint : waypoints)
    {
      if ((waypoint.fScore < leastScore || leastScoreWaypoint == null) && waypoint != waypointCurrent && waypoint.openList)
      {
        leastScore = waypoint.fScore;
        leastScoreWaypoint = waypoint;
      }
    }
    return leastScoreWaypoint;
  }

  private boolean checkClosedList(Set<Waypoint> waypoints, Waypoint waypointTarget)
  {
    for (Waypoint waypoint : waypoints)
    {
      if (waypoint == waypointTarget && waypoint.closedList)
      {
        return true;
      }
    }
    return false;
  }

  private void cleanupWaypoints(Set<Waypoint> waypoints)
  {
    for (Waypoint waypoint : waypoints)
    {
      waypoint.cleanup();
    }
  }

  private ArrayList<Waypoint> getPath(Set<Waypoint> waypoints, Waypoint waypointSource, Waypoint waypointTarget)
  {
    Waypoint waypointBacktrack = waypointTarget;
    ArrayList<Waypoint> path = new ArrayList<Waypoint>();
    while (true)
    {
      path.add(0, waypointBacktrack);
      waypointBacktrack = waypointBacktrack.parent;
      if (waypointBacktrack == waypointSource)
      {
        path.add(0, waypointBacktrack);
        cleanupWaypoints(waypointSet);
        return path;
      }
    }
  }
}

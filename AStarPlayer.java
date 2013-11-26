package airplane.g5;

import java.util.List;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Set;

import org.apache.log4j.Logger;

import airplane.sim.GameConfig;
import airplane.sim.Plane;
import airplane.sim.SimulationResult;

public class AStarPlayer  extends airplane.sim.Player
{

  private Logger logger = Logger.getLogger(this.getClass()); // for logging

  // Map from plane object to plane state object
  private List<PlaneState> planeStateMap;
  private List<PlaneState> planeStateMapSim;

  private ArrayList<Plane> sortedPlanes = new ArrayList<Plane>();
  private Set<Integer> flyingPlanes = new HashSet<Integer>();
  private Set<Integer> criticalPlanes = new HashSet<Integer>();
  
  private ArrayList<Zone> safetyZones = new ArrayList<Zone>();
  private Set<Route> routeSet = new HashSet<Route>();
  private Map<Route, Set<PlaneState> > routeMap = new HashMap<Route, Set<PlaneState> >();
  private Map<Route, Integer> routeDirectionMap = new HashMap<Route, Integer>();

  private static final double WAITING = -1;
  private static final double LANDED = -2;
  private static final double MAX_TURN = 9.99999999999999;
  private static final double CRITICAL_COLLISION_ZONE_ANGLE = 10;
  private static final double CRITICAL_COLLISION_ZONE_ANGLE2 = 90;
  private static final double COLLISION_ZONE_TARGET_RADIUS = .7;
  private static final double COLLISION_ZONE_RADIUS = 5.0;
  private static final double AIRPORT_ZONE_RADIUS = 1;
  //private static final int CRITICAL_ROUTE_TRAFFIC = 1; // interesting patterns
  private int CRITICAL_ROUTE_TRAFFIC = 5; // for flow performance
  private int CRITICAL_ROUTE_TRAFFIC2 = 2; // lower traffic threshold used for choosing between collision zone angles.
  private static final int CRITICAL_WAYPOINT_TRAFFIC = 10;
  static final double WAYPOINT_ZONE_RADIUS = 7;
  static final double WAYPOINT_ZONE_RADIUS2 = 13;
  private static final double LANDING_WAYPOINT_ZONE_RADIUS = 11;
  private static final double CRITICAL_WAYPOINT_SWITCH_ANGLE = 30;
  private static final double OPAQUE_MARGIN = 20;
  private static final double ADAPTIVE_TAKEOFF_ANGLE_RADIUS = 10;
  private static final double INVALID_TAKEOFF_ANGLE_RANGE = 10;
  private static final int BOARD_SIZE = 100;
  private static final double CRITICAL_SAFETY_DIST = 15; // distance a safety zone's center must be from a potentail path
  private static final double CRITICAL_ZONE_DIST = 35;
  private static final double SAFETY_ZONE_RADIUS = 5;

  private boolean testDepart = false;
  private int simStartRound = 0;
  private int simPlaneId = -1;

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
    simStartRound = round;
    simPlaneId = planeId;
    PlaneState planeState = planeStateMap.get(planeId);

    // get current flow routes
    Set<Route> currentFlowRoutes = new HashSet<Route> ();
    if (planeState.route.currentFlowRoutes != null)
    {
      currentFlowRoutes = planeState.route.currentFlowRoutes;
    }
    else
    {
      for (Route route : routeSet)
      {
        if (route.currentTraffic >= CRITICAL_ROUTE_TRAFFIC && route != planeStateMap.get(planeId).route)
        {
          currentFlowRoutes.add(route);
        }
      }
      planeState.route.currentFlowRoutes = currentFlowRoutes;
    }

    if (criticalPlanes.contains(planeId))
      return true;

    boolean depart = false;
    ArrayList<Plane> planesToSim = new ArrayList<Plane>();
    for (Plane p : planes)
      planesToSim.add(p);
    for (int i : criticalPlanes)
    {
      if (!planes.contains(planeStateMap.get(i).plane))
    	  planesToSim.add(planeStateMap.get(i).plane);
    }
    if (!criticalPlanes.contains(planeId))
    	planesToSim.add(planeStateMap.get(planeId).plane);

    boolean upgradeToCollision = false;
    Point2D collisionAvoidTarget = new Point2D.Double(0, 0);
    
    ArrayList<Waypoint> aStarPath = new ArrayList<Waypoint> ();
    HashSet<Line2D> aStarWalls = new HashSet<Line2D> ();
    double aStarZoneRadius = WAYPOINT_ZONE_RADIUS;
    boolean upgradeToAStar = false;
    double takeoffAngle = -1;

    boolean upgradeToSafetyZone = false;
    
    // refresh simulator state
    refreshSimState();

    // first try straight line path
    planeStateMapSim.get(planeId).state = PlaneState.States.NULL_STATE;
    SimulationResult result = startSimulation(planesToSim, round);
    if (result.getReason() == SimulationResult.NORMAL)
      depart = true;
    SimulationResult resultBackup = result;

    /*if (round >= 49 && (planeId == 43 || planeId == 44))
    {
      PlaneState planeState43 = planeStateMap.get(43);
      PlaneState planeState44 = planeStateMap.get(44);
      logger.info("round " + round);
    }*/

    if (result.getReason() != SimulationResult.NORMAL)
    {
      result = resultBackup;
    }
    if (result.getReason() == SimulationResult.TOO_CLOSE) // opposite collision avoidance
    {
      // check collision bearings of colliding planes:
      depart = false;
      PlaneState collisionPlaneState = getCollisionPlaneState(result.getPlanes(), planeId, GameConfig.SAFETY_RADIUS);
      Plane simPlane1 = planeStateMapSim.get(planeId).plane;
      Plane simPlane2 = collisionPlaneState.plane;
      Point2D simP1 = simPlane1.getLocation();
      Point2D simP2 = simPlane2.getLocation();
      double angleOfApproach = addBearings(simPlane1.getBearing(), - simPlane2.getBearing());
      double criticalCollisionZoneAngle = CRITICAL_COLLISION_ZONE_ANGLE2;
      if (collisionPlaneState.route.currentTraffic >= CRITICAL_ROUTE_TRAFFIC2)
      {
        criticalCollisionZoneAngle = CRITICAL_COLLISION_ZONE_ANGLE;
      }
      if ((angleOfApproach <= 180 + criticalCollisionZoneAngle) && (angleOfApproach >= 180 - criticalCollisionZoneAngle) && simPlane2.getBearing() >= 0)
      {
        PlaneState planeState1 = planeStateMap.get(simPlane1.id);
        PlaneState planeState2 = planeStateMap.get(simPlane2.id);
        Plane plane1 = planeState1.plane;
        Plane plane2 = planeState2.plane;
        Vector approachVector = new Vector(simPlane2.getBearing() - 90);
        Vector avoidVector = approachVector.rotate90AntiClockwise();
        avoidVector.multiply(COLLISION_ZONE_RADIUS);
        Vector avoidVectorOpposite = avoidVector.rotateOpposite();

        Vector locationVector = new Vector(plane2.getLocation());
        Vector approachVectorOpposite = approachVector.rotateOpposite();
        double axisDist = approachVector.dotProduct(new Vector(locationVector.getPoint(), plane1.getLocation()));
        approachVector.multiply(axisDist);
        Vector lineV1 = Vector.addVectors(locationVector, approachVector);
        approachVector.normalize();
        approachVector.multiply(150);
        approachVectorOpposite.multiply(150);
        Vector lineV2 = Vector.addVectors(locationVector, approachVectorOpposite); 

        Line2D axisLine = new Line2D.Double(lineV1.getPoint(), lineV2.getPoint());
        double k = axisLine.ptLineDist(plane1.getLocation());
        double c = (lineV1.getPoint().distance(simPlane2.getLocation()) + simPlane2.getLocation().distance(plane2.getLocation()));
        double x = ((k-COLLISION_ZONE_RADIUS)*(k-COLLISION_ZONE_RADIUS) + c*c)/(2*c);
        approachVectorOpposite.normalize();
        approachVector.normalize();
        approachVector.multiply(x);
        Vector collisionVector = Vector.addVectors(locationVector, approachVector);
        Vector avoidVectorAbsolute = Vector.addVectors(collisionVector, avoidVector);
        Vector avoidVectorAbsoluteOpposite = Vector.addVectors(collisionVector, avoidVectorOpposite);

        // refresh simulator state
        refreshSimState();
        planeStateMapSim.get(planeId).currentTarget = new Point2D.Double(avoidVectorAbsolute.x, avoidVectorAbsolute.y);
        planeStateMapSim.get(planeId).state = PlaneState.States.COLLISION_STATE;

        result = startSimulation(planesToSim, round);
        if (result.getReason() == SimulationResult.NORMAL)
        {
          depart = true;
          collisionAvoidTarget = planeStateMapSim.get(planeId).currentTarget;
          upgradeToCollision = true;
        }
        if (!upgradeToCollision)
        {
          // refresh simulator state
          refreshSimState();
          planeStateMapSim.get(planeId).currentTarget = new Point2D.Double(avoidVectorAbsoluteOpposite.x, avoidVectorAbsoluteOpposite.y);
          planeStateMapSim.get(planeId).state = PlaneState.States.COLLISION_STATE;

          result = startSimulation(planesToSim, round);
          if (result.getReason() == SimulationResult.NORMAL)
          {
            depart = true;
            collisionAvoidTarget = planeStateMapSim.get(planeId).currentTarget;
            upgradeToCollision = true;
          }
        }
        /*if (!upgradeToCollision)
        {
          x = ((k+COLLISION_ZONE_RADIUS)*(k+COLLISION_ZONE_RADIUS) + c*c)/(2*c);
          approachVector.normalize();
          approachVector.multiply(x);
          collisionVector = Vector.addVectors(locationVector, approachVector);
          avoidVectorAbsolute = Vector.addVectors(collisionVector, avoidVector);
          avoidVectorAbsoluteOpposite = Vector.addVectors(collisionVector, avoidVectorOpposite);

          // refresh simulator state
          refreshSimState();
          planeStateMapSim.get(planeId).currentTarget = new Point2D.Double(avoidVectorAbsoluteOpposite.x, avoidVectorAbsoluteOpposite.y);
          planeStateMapSim.get(planeId).state = PlaneState.States.COLLISION_STATE;

          result = startSimulation(planesToSim, round);
          if (result.getReason() == SimulationResult.NORMAL)
          {
            depart = true;
            collisionAvoidTarget = planeStateMapSim.get(planeId).currentTarget;
            upgradeToCollision = true;
          }
        }
        if (!upgradeToCollision)
        {
          // refresh simulator state
          refreshSimState();
          planeStateMapSim.get(planeId).currentTarget = new Point2D.Double(avoidVectorAbsolute.x, avoidVectorAbsolute.y);
          planeStateMapSim.get(planeId).state = PlaneState.States.COLLISION_STATE;

          result = startSimulation(planesToSim, round);
          if (result.getReason() == SimulationResult.NORMAL)
          {
            depart = true;
            collisionAvoidTarget = planeStateMapSim.get(planeId).currentTarget;
            upgradeToCollision = true;
          }
        }*/
      }
    }
    if (result.getReason() == SimulationResult.TOO_CLOSE && currentFlowRoutes.size() == 0) // if collision close to source airport, change takeoff angle
    {
      depart = false;
      double bearing = getLastTakeoffAngle(planeId);
      if (bearing != WAITING)
      {
        // refresh simulator state
        refreshSimState();
        planeStateMapSim.get(planeId).takeoffAngle = bearing;
        result = startSimulation(planesToSim, round);
        if (result.getReason() == SimulationResult.NORMAL)
        {
          takeoffAngle = bearing;
          depart = true;
        }
      }
      if (depart == false)
      {
        ArrayList<Double> bearings = getTakeoffAngle(result, planeId);
        if (bearings != null)
        {
          for (Double bearing1 : bearings)
          {
            // refresh simulator state
            refreshSimState();
            planeStateMapSim.get(planeId).takeoffAngle = bearing1;
            result = startSimulation(planesToSim, round);
            if (result.getReason() == SimulationResult.NORMAL)
            {
              takeoffAngle = bearing1;
              depart = true;
              break;
            }
          }
        }
      }
    }
    
    // check if intersecting with high density flow, take A* path in that case
    boolean intersectsFlow = false;
    if (result.getReason() == SimulationResult.NORMAL)
    {
      if(checkHighDensityFlowIntersection(planeStateMapSim.get(planeId).path, currentFlowRoutes, .5))
        intersectsFlow = true;
    }

    if (result.getReason() == SimulationResult.TOO_CLOSE || intersectsFlow) // A* routing if collision with a flow
    {
      depart = false;
      PlaneState collisionPlaneState = getCollisionPlaneState(result.getPlanes(), planeId, GameConfig.SAFETY_RADIUS);
      Plane simPlane1 = planeStateMapSim.get(planeId).plane;
      Plane simPlane2 = collisionPlaneState.plane;
      Point2D simP1 = simPlane1.getLocation();
      Point2D simP2 = simPlane2.getLocation();

      // traffic more than flow threshold and not this flow or intersects high density flow
      if (intersectsFlow || (planeStateMap.get(simPlane2.id).route.currentTraffic >= CRITICAL_ROUTE_TRAFFIC && planeStateMap.get(simPlane2.id).route != planeStateMap.get(simPlane1.id).route))
      {
        // update walls
        updateWalls(currentFlowRoutes);
        // update current waypoints
        waypointSet.clear();
        visibilityMap.clear();
        for (Route flow : currentFlowRoutes)
        {
          // initialize waypoint traffic maps if null
          if (flow.waypoint1.currentTrafficMap.get(planeState.route) == null)
          {
            flow.waypoint1.currentTrafficMap.put(planeState.route, new HashMap<Integer, Integer> ());
            Map<Integer, Integer> flowTraffic = flow.waypoint1.currentTrafficMap.get(planeState.route);
            flowTraffic.put(planeState.routeDirection, 0);
            flowTraffic.put(-planeState.routeDirection, 0);
          }
          if (flow.waypoint2.currentTrafficMap.get(planeState.route) == null)
          {
            flow.waypoint2.currentTrafficMap.put(planeState.route, new HashMap<Integer, Integer> ());
            Map<Integer, Integer> flowTraffic = flow.waypoint2.currentTrafficMap.get(planeState.route);
            flowTraffic.put(planeState.routeDirection, 0);
            flowTraffic.put(-planeState.routeDirection, 0);
          }
          // apply pruning on busy waypoints
          if (flow.waypoint1.currentTraffic - flow.waypoint1.currentTrafficMap.get(planeState.route).get(planeState.routeDirection) < CRITICAL_WAYPOINT_TRAFFIC)
            addWaypoint(flow.waypoint1);
          if (flow.waypoint2.currentTraffic - flow.waypoint2.currentTrafficMap.get(planeState.route).get(planeState.routeDirection) < CRITICAL_WAYPOINT_TRAFFIC)
            addWaypoint(flow.waypoint2);
        }
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
          /*if (result.getReason() == SimulationResult.TOO_CLOSE) // try different takeoff angle if collision near source airport
          {
            double tryTakeoffAngle = getLastTakeoffAngle(planeId);
            if(tryTakeoffAngle != WAITING)
            {
              // refresh simulator state
              refreshSimState();
              if (path.size() <= 2) // create landing zone
              {
                planeStateMapSim.get(planeId).zoneRadius = LANDING_WAYPOINT_ZONE_RADIUS;
              }
              planeStateMapSim.get(planeId).path = path;
              planeStateMapSim.get(planeId).state = PlaneState.States.ORBIT_STATE;
              planeStateMapSim.get(planeId).walls = walls;
              planeStateMapSim.get(planeId).takeoffAngle = tryTakeoffAngle;
              takeoffAngle = tryTakeoffAngle;
              result = startSimulation(planesToSim, round);
            }
            if (result.getReason() != SimulationResult.NORMAL)
            {
              ArrayList<Double> bearings = getTakeoffAngle(result, planeId);
              if(bearings != null)
              {
                for (Double bearing : bearings)
                {
                  // refresh simulator state
                  refreshSimState();
                  if (path.size() <= 2) // create landing zone
                  {
                    planeStateMapSim.get(planeId).zoneRadius = LANDING_WAYPOINT_ZONE_RADIUS;
                  }
                  planeStateMapSim.get(planeId).path = path;
                  planeStateMapSim.get(planeId).state = PlaneState.States.ORBIT_STATE;
                  planeStateMapSim.get(planeId).walls = walls;
                  planeStateMapSim.get(planeId).takeoffAngle = bearing;
                  takeoffAngle = bearing;
                  result = startSimulation(planesToSim, round);
                  if (result.getReason() == SimulationResult.NORMAL)
                    break;
                }
              }
            }
          }*/
          for (int i = 0; i < 1; i++) // n tries
          {
            if (result.getReason() == SimulationResult.TOO_CLOSE) // try larger zone radius 
            {
              PlaneState collisionPlaneState2 = getCollisionPlaneState(result.getPlanes(), planeId, GameConfig.SAFETY_RADIUS);
              Route collideRoute = collisionPlaneState2.route;
              PlaneState simPlaneState1 = planeStateMapSim.get(planeId);
              if ((collideRoute == planeState.route && planeState.routeDirection != collisionPlaneState2.routeDirection) ||
                  (collideRoute != planeState.route))
              {
                // refresh simulator state
                refreshSimState();
                if (path.size() <= 2) // create landing zone
                {
                  planeStateMapSim.get(planeId).zoneRadius = LANDING_WAYPOINT_ZONE_RADIUS;
                }
                planeStateMapSim.get(planeId).zoneRadius = WAYPOINT_ZONE_RADIUS2;
                planeStateMapSim.get(planeId).path = path;
                planeStateMapSim.get(planeId).state = PlaneState.States.ORBIT_STATE;
                planeStateMapSim.get(planeId).walls = walls;
                planeStateMapSim.get(planeId).takeoffAngle = takeoffAngle;
                result = startSimulation(planesToSim, round);
              }
              else if (collideRoute == planeState.route && routeSet.size() > 2) // abort if collision with same flow
              {
                break;
              }
            }
            if (result.getReason() != SimulationResult.NORMAL) // try pruning conflicting waypoint(s)
            {
              PlaneState collisionPlaneState2 = getCollisionPlaneState(result.getPlanes(), planeId, GameConfig.SAFETY_RADIUS);
              Route collideRoute = collisionPlaneState2.route;
              PlaneState simPlaneState1 = planeStateMapSim.get(planeId);
              // update current waypoints
              if (collideRoute != planeState.route || routeSet.size() <= 2)
                removeWaypoint(simPlaneState1.path.get(simPlaneState1.pathIter));
              path = AStar(new Waypoint(planeState.plane.getLocation()), new Waypoint(planeState.plane.getDestination()));
              if (path == null)
              {
                break;
                /*ArrayList<Zone> zones = new ArrayList<Zone>();
                zones.addAll(safetyZones);
                while (zones.size() > 0)
                {
                  Point2D.Double destination = planeState.plane.getDestination();
                  Zone nextZone = getClosestZone(destination, zones);
                  
                  // break if the next closest zone is too far
                  if (nextZone.center.distance(destination) >= CRITICAL_ZONE_DIST) 
                  {
                    break;
                  }
                  
                  path = AStar(new Waypoint(planeState.plane.getLocation()), new Waypoint(nextZone.center));
                  if (path != null)
                  {
                    break;
                  }
                  
                  zones.remove(nextZone);
                }*/
              }
              if (path != null)
              {
                // refresh simulator state
                refreshSimState();
                planeStateMapSim.get(planeId).path = path;
                planeStateMapSim.get(planeId).state = PlaneState.States.ORBIT_STATE;
                planeStateMapSim.get(planeId).walls = walls;
                planeStateMapSim.get(planeId).takeoffAngle = takeoffAngle;
                if (path.size() <= 2) // create landing zone
                {
                  planeStateMapSim.get(planeId).zoneRadius = LANDING_WAYPOINT_ZONE_RADIUS;
                }
                result = startSimulation(planesToSim, round);
              }
            }
            if (result.getReason() == SimulationResult.OUT_OF_BOUNDS) // prune current waypoint
            {
              PlaneState simPlaneState1 = planeStateMapSim.get(planeId);
              // update current waypoints
              removeWaypoint(simPlaneState1.path.get(simPlaneState1.pathIter));
            }
            if (result.getReason() == SimulationResult.NORMAL /*|| result.getReason() == SimulationResult.STOPPED*/)
            {
              depart = true;
              upgradeToAStar = true;
              aStarPath = path;
              aStarZoneRadius = planeStateMapSim.get(planeId).zoneRadius;
              takeoffAngle = planeStateMapSim.get(planeId).takeoffAngle;

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
                {
                  // initialize waypoint traffic map if null
                  if (waypoint.currentTrafficMap.get(planeState.route) == null)
                  {
                    waypoint.currentTrafficMap.put(planeState.route, new HashMap<Integer, Integer> ());
                    Map<Integer, Integer> flowTraffic = waypoint.currentTrafficMap.get(planeState.route);
                    flowTraffic.put(planeState.routeDirection, 0);
                    flowTraffic.put(-planeState.routeDirection, 0);
                  }
                  Map<Integer, Integer> flowTraffic = waypoint.currentTrafficMap.get(planeState.route);
                  int trafficValue = flowTraffic.get(planeState.routeDirection);
                  trafficValue++;
                  flowTraffic.put(planeState.routeDirection, trafficValue);
                  waypoint.currentTraffic++;
                }
              }
              break;
            }
          }
        }
        
        // A* failed, move to safety zone 
//        else 
//        {
//        	upgradeToSafetyZone = true;
//        }
      }

    }
    /*if (testDepart)
    {
      depart = false;
    }
    if (!result.isSuccess())
    {
      logger.info("COULDN'T DEPART BECAUSE " + reasonToString(result.getReason()));
      depart = false;
      if (result.getReason() == SimulationResult.STOPPED && !testDepart)
      {
        testDepart = true;
        depart = true;
      }
    }*/
    if (upgradeToCollision)
    {
      planeState.state = PlaneState.States.COLLISION_STATE;
      planeState.currentTarget = collisionAvoidTarget;
      if (planeState.route.lastPath == null)
        planeState.route.lastPath = planeStateMapSim.get(planeState.plane.id).path;
      if (!planeState.route.lastPathDirectional.containsKey(planeState.routeDirection))
        planeState.route.lastPathDirectional.put(planeState.routeDirection, planeStateMapSim.get(planeState.plane.id).path);
      planeState.route.lastTakeoffAngle.put(planeState.routeDirection, takeoffAngle);
    }
    else if (upgradeToAStar)
    {
      planeState.state = PlaneState.States.ORBIT_STATE;
      planeState.path = aStarPath;
      planeState.walls = aStarWalls;
      planeState.zoneRadius = aStarZoneRadius;
      planeState.takeoffAngle = takeoffAngle;
      planeState.route.lastPath = planeStateMapSim.get(planeState.plane.id).path;
      planeState.route.lastPathDirectional.put(planeState.routeDirection, planeStateMapSim.get(planeState.plane.id).path);
      planeState.route.lastTakeoffAngle.put(planeState.routeDirection, takeoffAngle);
    }
//    else if (upgradeToSafetyZone)
//    {
//    	Point2D.Double dest = planeState.plane.getDestination();
//    	Zone safetyZone = getClosestZone(dest);
//    	planeState.state = PlaneState.States.ORBIT_STATE;
//
//    	// update current waypoints
//    	waypointSet.clear();
//    	visibilityMap.clear();
//    	for (Route flow:currentFlowRoutes)
//    	{
//    		addWaypoint(flow.waypoint1);
//    		addWaypoint(flow.waypoint2);
//    	}
//
//    	// calculate A* path to safety zone
//    	ArrayList<Waypoint> safetyPath = AStar(new Waypoint(planeState.plane.getLocation()), new Waypoint(safetyZone.center));
//
//    	if (safetyPath == null)
//    	{
//    		planeState.state = PlaneState.States.NULL_STATE;
//    	    planeState.takeoffAngle = takeoffAngle;
//    		depart = false;
//    	}
//    	else 
//    	{
//        	planeState.path = safetyPath;
//        	aStarWalls = new HashSet<Line2D>();
//        	aStarWalls.addAll(walls);
//        	
//        	planeState.walls = aStarWalls;
//        	planeState.walls = walls;
//    	}
//    	planeState.zoneRadius = safetyZone.radius;
//    }
    else
    {
      planeState.state = PlaneState.States.NULL_STATE;
      planeState.takeoffAngle = takeoffAngle;
      if (planeState.route.lastPath == null)
        planeState.route.lastPath = planeStateMapSim.get(planeState.plane.id).path;
      if (!planeState.route.lastPathDirectional.containsKey(planeState.routeDirection))
        planeState.route.lastPathDirectional.put(planeState.routeDirection, planeStateMapSim.get(planeState.plane.id).path);
      planeState.route.lastTakeoffAngle.put(planeState.routeDirection, takeoffAngle);
    }
    return depart;
  }
  
  public Zone getClosestZone(Point2D.Double p, ArrayList<Zone> zones)
  {
	  double minDist = (double)Integer.MAX_VALUE;
	  Zone closestZone = null;
	  for (Zone z : zones)
	  {
		  double currDist = z.center.distance(p);
		  if (currDist < minDist)
		  {
			  minDist = currDist;
			  closestZone = z;
		  }
	  }
	  
	  return closestZone;
  }

  public boolean checkHighDensityFlowIntersection(ArrayList<Waypoint> path, Set<Route> currentFlowRoutes, double minDensity)
  {
    Set<Line2D> pathWalls = getFlowWalls(path);
    Set<Route> highDensityFlows = new HashSet<Route> ();
    for (Route flow : currentFlowRoutes)
    {
      if (getFlowDensity(flow) >= minDensity)
      {
        highDensityFlows.add(flow);
      }
    }
    // update walls
    walls.clear();
    walls = getFlowWalls(highDensityFlows);
    for (Line2D pathWall : pathWalls)
    {
      if (!isInLineOfSight(pathWall.getP1(), pathWall.getP2()))
      {
        return true;
      }
    }
    return false;
  }

  public double getFlowDensity(Route route)
  {
    return (route.currentTraffic * 5) / getPathLength(route.lastPath);
  }

  public PlaneState getCollisionPlaneState(ArrayList<Plane> simPlanes, int testPlaneId, double safetyRadius)
  {
    Plane simPlane1 = planeStateMapSim.get(testPlaneId).plane;
    Point2D simP1 = simPlane1.getLocation();

    for (Plane simPlane2: simPlanes)
    {
      Point2D simP2 = simPlane2.getLocation();
      if (!simPlane1.equals(simPlane2) && simPlane1.getBearing() != -2 && simPlane1.getBearing() != -1 && simPlane2.getBearing() != -2 && simPlane2.getBearing() != -1 && simP1.distance(simP2) < safetyRadius)
      {
        return planeStateMapSim.get(simPlane2.id);
      }
    }
    //logger.warn("shouldn't be here!");
    return planeStateMapSim.get(testPlaneId);
  }

  public ArrayList<Double> getTakeoffAngle(SimulationResult result, int planeId)
  {
    PlaneState simPlaneState1 = planeStateMapSim.get(planeId);
    PlaneState planeState = planeStateMap.get(planeId);
    if (planeState.plane.getLocation().distance(simPlaneState1.plane.getLocation()) <= ADAPTIVE_TAKEOFF_ANGLE_RADIUS)
    {
      PlaneState simPlaneState2 = getCollisionPlaneState(result.getPlanes(), planeId, GameConfig.SAFETY_RADIUS);
      PlaneState planeState2 = planeStateMap.get(simPlaneState2.plane.id);
      if (simPlaneState2.route != simPlaneState1.route || (simPlaneState2.route == simPlaneState1.route && simPlaneState2.routeDirection != simPlaneState1.routeDirection))
      {
        double bearing = simPlaneState2.plane.getBearing();
        double destBearing = calculateBearing(planeState.plane.getLocation(), planeState.plane.getDestination());
        double deltaBearing = addBearings(destBearing, -bearing);
        if ((deltaBearing <= 180 - INVALID_TAKEOFF_ANGLE_RANGE && deltaBearing >= 0) || (deltaBearing >= 180 + INVALID_TAKEOFF_ANGLE_RANGE && deltaBearing <= 360) || planeState2.route.currentTraffic >= CRITICAL_ROUTE_TRAFFIC)
        {
          bearing = bearing;
        }
        else
        {
          bearing = WAITING;
        }
        ArrayList<Double> bearings = new ArrayList<Double> ();
        double newBearing = destBearing;
        /*double sign = 1;
        if (destBearing < 180)
          sign = -1;*/
        double sign = -1;
        if (bearing != WAITING)
        {
          while(true)
          {
            if (Math.abs(addBearings(bearing, -newBearing)) <= 10)
            {
              bearings.add(bearing);
              break;
            }
            newBearing = addBearings(newBearing, sign*10);
            bearings.add(newBearing);
          }
        }
        return bearings;
      }
    }
    return null;
  }

  public double getLastTakeoffAngle(int planeId)
  {
    PlaneState planeState =  planeStateMap.get(planeId);
    double lastTakeoffAngle = WAITING;
    if (planeState.route.lastTakeoffAngle.containsKey(planeState.routeDirection))
      lastTakeoffAngle = planeState.route.lastTakeoffAngle.get(planeState.routeDirection);
    return lastTakeoffAngle;
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
  
  public void calculateSafetyZones(ArrayList<Plane> planes)
  {
	 // calculate all possible paths a plane may travel
	 ArrayList<Line2D.Double> paths = new ArrayList<Line2D.Double>();
	 Set<Point2D.Double> airports = new HashSet<Point2D.Double>();
	 
	 for (Plane p :planes) 
	 {
		 Line2D.Double nextPath = new Line2D.Double(p.getLocation(), p.getDestination());
		 paths.add(nextPath);
		 airports.add(p.getDestination());
	 }
	 
	 for (Point2D.Double p1: airports) 
	 {
		 for (Point2D.Double p2: airports) 
		 {
			 if (!p1.equals(p2)) 
			 {
				 Line2D.Double airportPath = new Line2D.Double(p1, p2);
				 Line2D.Double reversePath = new Line2D.Double(p2, p1);
				 if (!paths.contains(reversePath) && !paths.contains(airportPath)) {
					 paths.add(airportPath);
				 }
			 }
		 }
	 }
	 
/* boolean[][] invalidCenter = new boolean[BOARD_SIZE][BOARD_SIZE];
	 for (int i = (int)SAFETY_ZONE_RADIUS; i < BOARD_SIZE - SAFETY_ZONE_RADIUS; i++)
	 {
		 for (int j = (int)SAFETY_ZONE_RADIUS; j < BOARD_SIZE - SAFETY_ZONE_RADIUS; j++)
		 {
			 if (!invalidCenter[i][j]) {
				 Point2D.Double potentialCenter = new Point2D.Double(i,j);
				 boolean invalid = false;
				 for (Line2D.Double path : paths)
				 {
					 if (path.ptLineDist(potentialCenter) < CRITICAL_SAFETY_DIST)
					 {
						 invalid = true;
						 break;
					 }
				 }
				 
				 if (!invalid)
				 {
					 Zone nextSafetyZone = new Zone(potentialCenter, SAFETY_ZONE_RADIUS);
					 safetyZones.add(nextSafetyZone);
					 int safetySpan = 15; // next zone center must be at least 15 away from this one to avoid collisions
					 int startRow = Math.max(0, i-safetySpan);
					 int startCol = Math.max(0, j-safetySpan);
					 
					 // invalidate centers within this zone
					 for (int k = startRow; k < 2 * safetySpan + Math.min(0, i-safetySpan); k++) 
					 {
						 for (int l = startCol; l < 2 * safetySpan + Math.min(0, j-safetySpan); l++)
						 {
							 invalidCenter[k][l] = true;
						 }
					 }
					 
					 // move to the next possible zone center along this row
					 j+= safetySpan - 1;
				 }
			 }
		 }
	 }*/
	 
	 double zoneRadius = 7.5;
	 
	 for (int i = 0; i < BOARD_SIZE; i+=15)
	 {
		 for (int j = 0; j < BOARD_SIZE; j+=15)
		 {
			Point2D.Double potentialCenter = new Point2D.Double(i + zoneRadius, j + zoneRadius); 
			 boolean invalid = false;
			 for (Line2D.Double path : paths)
			 {
				 if (path.ptLineDist(potentialCenter) < CRITICAL_SAFETY_DIST)
				 {
					 invalid = true;
					 break;
				 }
			 }
			 
			 if (!invalid)
			 {
				 Zone nextSafetyZone = new Zone(potentialCenter, SAFETY_ZONE_RADIUS);
				 safetyZones.add(nextSafetyZone);
			 }
		 }
	 }

  }

  public void updateWalls(Set<Route> currentFlowRoutes)
  {
    walls.clear();
    for (Route flow : currentFlowRoutes)
    {
      walls.add(new Line2D.Double(flow.waypoint1.point, flow.waypoint2.point));
    }
  }

  public Set<Line2D> getFlowWalls(Set<Route> currentFlowRoutes)
  {
    Set<Line2D> flowWalls = new HashSet<Line2D> ();
    // iterate over each route, add walls along paths  
    for (Route route: currentFlowRoutes)
    {
      flowWalls.addAll(getFlowWalls(route.lastPath));
    }
    return flowWalls;
  }

  public Set<Line2D> getFlowWalls(ArrayList<Waypoint> path)
  {
    Set<Line2D> flowWalls = new HashSet<Line2D> ();
    for (int i = 0; i < path.size() - 1; i++)
    {
      Waypoint wp1 = path.get(i);
      Waypoint wp2 = path.get(i + 1);
      Line2D wall = new Line2D.Double(wp1.point, wp2.point);
      flowWalls.add(wall);
    }
    return flowWalls;
  }

  public double getPathLength(ArrayList<Waypoint> path)
  {
    double pathLength = 0;
    for (int i = 0; i < path.size() - 1; i++)
    {
      Waypoint wp1 = path.get(i);
      Waypoint wp2 = path.get(i + 1);
      pathLength = pathLength + wp1.point.distance(wp2.point);
    }
    return pathLength;
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
    
    sortedPlanes.addAll(planes);
    Collections.sort(sortedPlanes, INCREASING_DEPT);
    Collections.sort(sortedPlanes, DECREASING_DIST);
    //Collections.sort(sortedPlanes, DECREASING_COST);
    
    // initialize ids to distinguish planes
    for (int i = 0; i < planes.size(); i++)
    {
      Plane p = planes.get(i);
      p.id = i;
      planes.set(i, p);
      PlaneState planeState = new PlaneState(p);
      planeStateMap.add(p.id, planeState);
    }

    // compute routes
    for (PlaneState planeState : planeStateMap)
    {
      Point2D p1 = planeState.plane.getLocation();
      Point2D p2 = planeState.plane.getDestination();
      planeState.routeDirection = Route.FORWARD;
      // check if this route can be clubbed with an existing route
      Route route = new Route(new Waypoint(p1), new Waypoint(p2));
      Set<PlaneState> routePlanes = new HashSet<PlaneState> ();
      for (Route routeMerge : routeSet)
      {
        Point2D wpP1 = routeMerge.waypoint1.point;
        Point2D wpP2 = routeMerge.waypoint2.point;

        if (wpP1.distance(p1) < AIRPORT_ZONE_RADIUS)
        {
          if (wpP2.distance(p2) < AIRPORT_ZONE_RADIUS)
          {
            planeState.routeDirection = Route.FORWARD;
            route = routeMerge;
            routePlanes = routeMap.get(route);
            if (routeDirectionMap.get(route) != Route.FORWARD)
              routeDirectionMap.put(route, 0);
          }
        }
        if (wpP1.distance(p2) < AIRPORT_ZONE_RADIUS)
        {
          if (wpP2.distance(p1) < AIRPORT_ZONE_RADIUS)
          {
            planeState.routeDirection = Route.BACKWARD;
            route = routeMerge;
            routePlanes = routeMap.get(route);
            if (routeDirectionMap.get(route) != Route.BACKWARD)
              routeDirectionMap.put(route, 0);
          }
        }
      }
      routeSet.add(route);
      planeState.route = route;
      routePlanes.add(planeState);
      if (!routeMap.containsKey(route))
        routeMap.put(route, routePlanes);
      if (!routeDirectionMap.containsKey(route))
        routeDirectionMap.put(route, planeState.routeDirection);
    }
    
    calculateSafetyZones(planes);

    ArrayList<Plane> simPlanes = new ArrayList<Plane> ();
    Map<Route, Integer> routesSeen = new HashMap<Route,Integer> ();
    Set<Route> routesDone = new HashSet<Route> ();
    // simulate trajectory of high cost planes, add as many as possible to set of critical planes
    for (Plane plane : sortedPlanes)
    {
      // simulate each route in each direction only once
      PlaneState planeState = planeStateMap.get(plane.id);
      Route route = planeState.route;
      if (routesDone.contains(route))
        continue;
      if (routeSet.size() == routesDone.size())
        break;
      if (!routesSeen.containsKey(route))
      {
        routesSeen.put(route, planeState.routeDirection);
      }
      else
      {
        int direction = routesSeen.get(route);
        if (direction == 0)
          continue;
        else if (direction != planeState.routeDirection)
        {
          routesSeen.put(route, 0);
          routesDone.add(route);
        }
        else if (routeDirectionMap.get(route) == planeState.routeDirection)
        {
          routesDone.add(route);
          continue;
        }
      }
      if (depart(plane.id, 0, simPlanes))
      {
        criticalPlanes.add(plane.id);
        simPlanes.add(plane);
      }
    }
    /*for (Plane plane : sortedPlanes)
    {
      if (depart(plane.id, 0, simPlanes))
      {
        criticalPlanes.add(plane.id);
        simPlanes.add(plane);
      }
    }*/
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
  
  private Comparator<Plane> DECREASING_DIST = new Comparator<Plane>()
  {
    public int compare(Plane p1, Plane p2)
    {
      double dist1 = p1.getLocation().distance(p1.getDestination());
      double dist2 = p2.getLocation().distance(p2.getDestination());
      if (dist1 < dist2)
      {
        return 1;
      }
      else if (dist2 < dist1)
      {
        return -1;
      }
      else
      {
        return 0;
      }
    }
  };

  /*
   * This is used when you're running your own simulation
   */
  @Override
  protected double[] simulateUpdate(ArrayList<Plane> planes, int round, double[] bearings)
  {
    if (round - simStartRound > 200)
    {
      stopSimulation();
    }
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
          double wpBearing = calculateBearing(p.getLocation(), (Point2D.Double) wpPoint);
          boolean switchCond1 = true;
          if (planeState.path.size() > 2)
          {
            Point2D wpPointNext = planeState.path.get(planeState.pathIter + 1).point;
            double wpNextBearing = calculateBearing(p.getLocation(), (Point2D.Double) wpPointNext);
            double wpRelativeBearing = addBearings(wpNextBearing, -wpBearing);
            double deltaBearing = addBearings(wpNextBearing, -p.getBearing());
            switchCond1 = deltaBearing <= CRITICAL_WAYPOINT_SWITCH_ANGLE || 
                          deltaBearing >= 360 - CRITICAL_WAYPOINT_SWITCH_ANGLE ||
                          (wpRelativeBearing >= 90 && wpRelativeBearing <= 270);
          }
          boolean switchCond2 = p.getLocation().distance(wpPoint) <= (planeState.zoneRadius + .1);
          if (switchCond1 && switchCond2)
          {
            planeState.pathIter++;
            if (planeState.pathIter >= planeState.path.size() - 1) // last waypoint
            {
              planeState.pathIter = planeState.path.size() - 1;
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
        if (p.getBearing() != -2 && p.getDepartureTime() <= round)
        {
          //logger.info("planeId: " + p.id + " sim departure time: " + round);
          if (planeState.state == PlaneState.States.NULL_STATE)
          {
            if (planeState.takeoffAngle == -1)
            {
              bearings[i] = straightLinePath(planeState);
            }
            else
            {
              straightLinePath(planeState);
              planeState.state = PlaneState.States.SPIRAL_STATE;
              bearings[i] = planeState.takeoffAngle;
              //logger.info("takeoff at sim: " + planeState.takeoffAngle + " round: " + round);
            }
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
            if (planeState.takeoffAngle == -1)
              bearings[i] = bearing;
            else
              bearings[i] = planeState.takeoffAngle;
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
    for (int i = 0; i < planes.size(); i++)
    {
//      Plane p = planes.get(i);
      Plane p = sortedPlanes.get(i);
      PlaneState planeState = planeStateMap.get(p.id);
      if (flyingPlanes.contains(p.id) && p.getBearing() != -2)
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
//          bearings[i] = spiralOrbit(planeState, p.getDestination());
        	bearings[p.id] = spiralOrbit(planeState, p.getDestination());
        }
        else if (planeState.state == PlaneState.States.ORBIT_STATE)
        {
          // check if next waypoint visible
          Point2D wpPoint = planeState.path.get(planeState.pathIter).point;
          double wpBearing = calculateBearing(p.getLocation(), (Point2D.Double) wpPoint);
          boolean switchCond1 = true;
          if (planeState.path.size() > 2)
          {
            Point2D wpPointNext = planeState.path.get(planeState.pathIter + 1).point;
            double wpNextBearing = calculateBearing(p.getLocation(), (Point2D.Double) wpPointNext);
            double wpRelativeBearing = addBearings(wpNextBearing, -wpBearing);
            double deltaBearing = addBearings(wpNextBearing, -p.getBearing());
            switchCond1 = deltaBearing <= CRITICAL_WAYPOINT_SWITCH_ANGLE || 
                          deltaBearing >= 360 - CRITICAL_WAYPOINT_SWITCH_ANGLE ||
                          (wpRelativeBearing >= 90 && wpRelativeBearing <= 270);
          }
          boolean switchCond2 = p.getLocation().distance(wpPoint) <= (planeState.zoneRadius + .1);
          if (switchCond1 && switchCond2)
          {
            planeState.pathIter++;
            if (planeState.pathIter >= planeState.path.size() - 1) // last waypoint
            {

              planeState.pathIter = planeState.path.size() - 1;
              bearings[p.id] = spiralOrbit(planeState, p.getDestination());

            }
            else // join next waypoint
            {
              wpPoint = planeState.path.get(planeState.pathIter).point;
//              bearings[i] = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, planeState.orbitDirection);
              bearings[p.id] = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, planeState.orbitDirection);
            }
          }
          else
          {
            // Move in orbital tangent towards waypoint zone
//            bearings[i] = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, planeState.orbitDirection);
            bearings[p.id] = joinOrbit(planeState, (Point2D.Double) wpPoint, planeState.zoneRadius, planeState.orbitDirection);
          }
        }
      }

      else if (round >= p.getDepartureTime() && p.getBearing() != -2)
      {
        boolean departCond = false;
        /*ArrayList<Plane> simPlanes = new ArrayList<Plane> ();
        for (int j : flyingPlanes)
        {
          simPlanes.add(planeStateMap.get(j).plane);
        }
        departCond = depart(p.id, round-1, simPlanes);*/
        ArrayList<Plane> simPlanes = new ArrayList<Plane> ();
        for (int j : flyingPlanes)
        {
          simPlanes.add(planeStateMap.get(j).plane);
        }
        departCond = depart(p.id, round-1, simPlanes);
        //logger.info("planeId: " + p.id + " real departure time: " + round);
        if (departCond && p.getBearing() != -2)
        {
          if (planeState.state == PlaneState.States.NULL_STATE)
          {
            if (planeState.takeoffAngle == -1)
            {
              bearings[p.id] = straightLinePath(planeState);
            }
            else
            {
              straightLinePath(planeState);
              planeState.state = PlaneState.States.SPIRAL_STATE;
              bearings[p.id] = planeState.takeoffAngle;
              //logger.info("takeoff: " + planeState.takeoffAngle + " round: " + round);
            }
            logger.debug("plane: " + i + " taking off in NULL_STATE, angle: " + bearings[p.id]);
          }
          else if (planeState.state == PlaneState.States.COLLISION_STATE)
          {
            // Move in orbital tangent
//            bearings[i] = collisionOrbit(planeState, (Point2D.Double) planeState.currentTarget);
        	  bearings[p.id] = collisionOrbit(planeState, (Point2D.Double) planeState.currentTarget);
            logger.debug("plane: " + i + " taking off in COLLISION_STATE, target: " + planeState.currentTarget.getX() + ", " + planeState.currentTarget.getY());
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
            
            if (planeState.takeoffAngle == -1)
              bearings[p.id] = bearing;
            else
              bearings[p.id] = planeState.takeoffAngle;
            logger.debug("plane: " + i + " taking off in ORBIT_STATE, angle: " + bearings[p.id]);
          }
          planeState.route.currentTraffic++;
          if (!flyingPlanes.contains(p.id))
          {
            flyingPlanes.add(p.id);
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
              {
                Map<Integer, Integer> flowTraffic = waypoint.currentTrafficMap.get(planeState.route);
                int trafficValue = flowTraffic.get(planeState.routeDirection);
                trafficValue++;
                flowTraffic.put(planeState.routeDirection, trafficValue);
                waypoint.currentTraffic--;
              }
            }
          }
        }
        flyingPlanes.remove(p.id);
      }
    }
    // update route flows
    for (Route route : routeSet)
    {
      if (route.currentTraffic == 0)
        route.currentFlowRoutes = null;
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
    directionVector.multiply(plane.getLocation().distance(center));
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
    if(plane.getLocation().distance(planeState.currentTarget) < COLLISION_ZONE_TARGET_RADIUS)
      return true;
    else
      return false;
  }

  public double straightLinePath(PlaneState planeState)
  {
    planeState.state = PlaneState.States.NULL_STATE;
    double bearing = calculateBearing(planeState.plane.getLocation(), planeState.plane.getDestination());
    // set path
    ArrayList<Waypoint> path = new ArrayList<Waypoint> ();
    path.add(0, new Waypoint(planeState.plane.getLocation()));
    path.add(1, new Waypoint(planeState.plane.getDestination()));
    planeState.path = path;
    return bearing;
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
    // set path
    ArrayList<Waypoint> path = new ArrayList<Waypoint> ();
    path.add(0, new Waypoint(planeState.plane.getLocation()));
    path.add(1, new Waypoint(planeState.plane.getDestination()));
    planeState.path = path;
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
    if (planeState.plane.getBearing() == WAITING && planeState.takeoffAngle != WAITING)
      return planeState.takeoffAngle;
    else if (planeState.plane.getBearing() == WAITING)
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
    {
      double radialBearing = calculateBearing((Point2D.Double)center, plane.getLocation());
      return radialBearing;
    }
    else
    {
      double radians = Math.asin(sin);
      double relativeDegree = Math.toDegrees(radians);
      double referenceDegree = calculateBearing(plane.getLocation(), (Point2D.Double)center);
      double degree = addBearings(referenceDegree, direction * relativeDegree);
      return degree;
    }
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

  void removeWaypoint(Waypoint waypoint)
  {
    removeFromVisibilityMap(waypoint);
    waypointSet.remove(waypoint);
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

  private void removeFromVisibilityMap(Waypoint waypoint1)
  {
    Set<Waypoint> waypointSet1 = visibilityMap.get(waypoint1);
    if (waypointSet1 != null)
    {
      for (Waypoint waypoint2 : waypointSet1)
      {
        Set<Waypoint> waypointSet2 = visibilityMap.get(waypoint2);
        waypointSet2.remove(waypoint1);
      }
      visibilityMap.remove(waypoint1);
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

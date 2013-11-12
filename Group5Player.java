package airplane.g5;

import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.HashSet;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

import org.apache.log4j.Logger;

import airplane.sim.Plane;
import airplane.sim.SimulationResult;



public class Group5Player extends airplane.sim.Player
{

  private Logger logger = Logger.getLogger(this.getClass()); // for logging

  // Map from plane object to plane state object
  private Map<Plane, PlaneState> planeStateMap;
  private PlaneState[] planeStateArray;

  // Temporary for demo
  private static double standardZoneRadius = 15;

  // Zones
  private Set<Zone> zones;

  // Simulation
  private static boolean simulationMode = false;
  private static boolean setTrajectory = false;

  private static final double WAITING = -1;
  private static final double LANDED = -2;
  private static final double MAX_TURN = 9.99999999999999;

  @Override
  public String getName()
  {
    return "ATC tower";
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

    planeStateMap = new HashMap<Plane, PlaneState> ();
    planeStateArray = new PlaneState[planes.size() + 1];
    zones = new HashSet<Zone> ();
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      PlaneState planeState = new PlaneState(plane);
      planeStateMap.put(plane, planeState);
      planeStateArray[i] = planeState;
    }

  }

  private static boolean isTooClose(Plane p1, ArrayList<Plane> planes) {
	  boolean tooClose = false;
	  Point2D.Double currLoc = p1.getLocation();
	  for (Plane p2 : planes) {
		  if (!p1.equals(p2) && currLoc.distance(p2.getLocation()) <= 5) {
			  tooClose = true;
			  break;
		  }
	  }
	  
	  return tooClose;
  }
  
  /*
   * This is called at each step of the simulation.
   * The List of Planes represents their current location, destination, and current
   * bearing; the bearings array just puts these all into one spot.
   * This method should return an updated array of bearings.
   */
  @Override
  public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearingArrayOrig)
  {
    // initialize bearings to original bearings
    double[] bearings = new double[planes.size()];
    for (int i = 0; i < bearingArrayOrig.length; i++)
    {
      bearings[i] = bearingArrayOrig[i];
    }
    
    for (Zone zone : zones)
    {
      zone.landingLock = false;
    }
    // update landing locks
    for (int i = 0; i < planes.size(); i++)
    {
      PlaneState planeState = planeStateArray[i];
      if (planeState.landingLock == true)
      {
        planeState.currentZone.landingLock = true;
      }
    }

    // main decision loop iterating over each plane
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      PlaneState planeState = planeStateArray[i];
      double bearingOrig = bearingArrayOrig[i];
      planeState.bearingOrig = bearingOrig;
      if (plane.getDepartureTime() > round || isTooClose(plane, planes))
      {
        // departure time in future: skip
        continue;
      }

      // state clause
      if (planeState.state == PlaneState.States.NULL_STATE)
      {
        // set up trajectory by running simulation
        /*if (!simulationMode)
        {
          simulationMode = true;
          setTrajectory = false;
          if (setTrajectory == false)
          {
            planeState.simulationTrajectory = LINE;
            SimulationResult result = startSimulation(planes, round);
            planeState.trajectoryAcquired = true;
          }
          if (!result == SimulationResult.NORMAL)
          {
            setTrajectory = false;
          }
          else
          {
            setTrajectory = true;
          }
          if (setTrajectory == false)
          {
            planeState.simulationTrajectory = SPIRAL;
            result = startSimulation(planes, round);
            planeState.trajectoryAcquired = true;
            if (!result == SimulationResult.NORMAL)
            {
              setTrajectory = false;
            }
          }
          else
          {
            setTrajectory = true;
          }

          else
          {
            setTrajectory = true;
          }

          simulationMode = false;
        }

        if (planeState.trajectoryAcquired && (setTrajectory == true || simulationMode))
        {*/
          if (/*planeState.simulationTrajectory == LINE && */acquireLandingLock(planeState))
          {
            double bearing = spiralOrbit(planeState, plane.getDestination());
            bearings[i] = bearing;
          }
          else /*if (planeState.simulationTrajectory == ORBIT)*/
          {
            double bearing = joinOrbit(planeState, plane.getDestination(), standardZoneRadius);
            bearings[i] = bearing;
          }
        //}
      }
      else if (planeState.state == PlaneState.States.ORBIT_STATE)
      {
        // acquire landing lock
        if (acquireLandingLock(planeState))
        {
          double bearing = leaveOrbit(planeState, plane.getDestination());
          bearings[i] = bearing;
        }
        else
        {
          double bearing = getOrbitBearing(planeState, plane.getDestination(), standardZoneRadius);
          bearings[i] = bearing;
        }
      }
      else if (planeState.state == PlaneState.States.SPIRAL_STATE)
      {
        if (planeState.bearingOrig != LANDED)
        {
          bearings[i] = spiralOrbit(planeState, plane.getDestination());
        }
        else
        {
          freeLandingLock(planeState);
        }
      }
    }

    return bearings;
  }

  public void setZone(PlaneState planeState, Point2D center, double radius)
  {
    boolean zoneFound = false;
    for (Zone zone : zones)
    {
      if (zone.center.distance(center) < zone.radius)
      {
        zoneFound = true;
        planeState.currentZone = zone;
      }
    }
    if (!zoneFound)
    {
      Zone zone = new Zone(center, radius);
      Zone zoneSelect;
      if (checkValidZone(zone) == zone)
        zones.add(zone);
      planeState.currentZone = zone;
    }
  }

  public Zone checkValidZone(Zone zone1)
  {
    for(Zone zone2 : zones)
    {
      if (zone1.center.distance(zone2.center) < zone1.radius + zone2.radius)
        return zone2;
    }
    return zone1;
  }

  public boolean acquireLandingLock(PlaneState planeState)
  {
    Plane plane = planeState.plane;
    setZone(planeState, plane.getDestination(), standardZoneRadius);
    if (planeState.currentZone.landingLock == false)
    {
      planeState.landingLock = true;
      planeState.currentZone.landingLock = true;
      return true;
    }
    else
      return false;
  }

  public boolean freeLandingLock(PlaneState planeState)
  {
    if (planeState.landingLock == true)
    {
      planeState.landingLock = false;
      planeState.currentZone.landingLock = false;
      return true;
    }
    return false;
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

  public double joinOrbit(PlaneState planeState, Point2D.Double center, double radius)
  {
    planeState.state = PlaneState.States.ORBIT_STATE;
    double bearing = getOrbitBearing(planeState, center, radius);

    Plane plane = planeState.plane;
    Vector tangent = new Vector(1, 0);
    Vector radial = tangent.rotate(180 - bearing);
    radial.multiply((float)radius);
    Vector centerVec = new Vector(plane.getLocation(), center);
    tangent = Vector.addVectors(centerVec, radial);
    Vector locationVector = new Vector(plane.getLocation());
    Vector tangentAbsolute = Vector.addVectors(locationVector, tangent);
    planeState.path = new Trajectory(new Line2D.Double(plane.getLocation(), tangentAbsolute.getPoint()));
    

    return bearing;
  }

  public double leaveOrbit(PlaneState planeState, Point2D.Double destination)
  {
    return spiralOrbit(planeState, destination);
  }

  public double spiralOrbit(PlaneState planeState, Point2D.Double destination)
  {
    planeState.state = PlaneState.States.SPIRAL_STATE;
    Plane plane = planeState.plane;
    Line2D path = new Line2D.Double(plane.getLocation(), destination);
    planeState.path = new Trajectory(path);
    double bearingDest = calculateBearing(plane.getLocation(), destination);
    if (planeState.bearingOrig == WAITING)
        return bearingDest;
    double bearingOrig = planeState.bearingOrig;
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

  public double getOrbitBearing(PlaneState planeState, Point2D.Double center, double radius)
  {
    Plane plane = planeState.plane;
    double dist = plane.getLocation().distance(center);
    double sin = radius / dist;
    if (sin > 1) // inside the circle
      return -1;
    double radians = Math.asin(sin);
    double relativeDegree = Math.toDegrees(radians);
    double referenceDegree = calculateBearing(plane.getLocation(), center);
    double degree = addBearings(referenceDegree, -relativeDegree);
    return degree;
  }

}

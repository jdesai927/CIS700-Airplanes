package airplane.g5;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Line2D.Double;

import org.apache.log4j.Logger;

import airplane.sim.Plane;
import airplane.sim.SimulationResult;


public class DynamicZonePlayer extends airplane.sim.Player
{

  private Logger logger = Logger.getLogger(this.getClass()); // for logging

  // Map from plane object to plane state object
  private Map<Plane, PlaneState> planeStateMap;
  private PlaneState[] planeStateArray;
  
  // Temporary for demo
  private static double radius = 15;
  private static boolean landingLock = false;

  private static final double WAITING = -1;
  private static final double LANDED = -2;
  private static final double MAX_TURN = 9.99999999999999;

  @Override
  public String getName()
  {
    return "Dynamic Zones";
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
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      
      // add id for identification in sim
      plane.id = i;
      planes.set(i, plane);
      
      PlaneState planeState = new PlaneState(plane);
      planeStateMap.put(plane, planeState);
      planeStateArray[i] = planeState;
    }
    
    changeTrajectory(planes.get(0), 0, planes);
  }
  
  
/*
 * returns true if any plane is less than 5 units away from the current plane, false otherwise
 */
  private static boolean isTooClose(Plane p1, ArrayList<Plane> planes) {
	  boolean tooClose = false;
	  Point2D.Double currLoc = p1.getLocation();
	  for (Plane p2 : planes) 
	  {
		  if (!p1.equals(p2) && currLoc.distance(p2.getLocation()) <= 5) 
		  {
			  tooClose = true;
			  break;
		  }
	  }
	  
	  return tooClose;
  }

  


private static Point2D.Double lineIntersect(Line2D l1, Line2D l2)
{
	double x11 = l1.getX1();
	double x12 = l1.getX2();
	
	double y11 = l1.getY1();
	double y12 = l1.getY2();
	
	double x21 = l2.getX1();
	double x22 = l2.getX2();
	
	double y21 = l2.getY1();
	double y22 = l2.getY2();
	
	double denom = (y22 - y21) * (x12 - x11) - (x22 - x21) * (y12 - y11);
	
	// no intersection, lines are parallel
	if (Math.abs(denom) < 0.001) 
	{
		return null;
	}
	
	double ua = ((x22 - x21) * (y11 - y21) - (y22 - y21) * (x11 - x21))/denom;
	double ub = ((x12 - x11) * (y11 - y21) - (y12 - y11) * (x11 - x21))/denom;
	if(ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f){
		return new Point2D.Double(x11 + ua*(x12 - x11), y11 + ub*(y12 - y11));
	}
	return null;
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
    
    landingLock = false;
    // update landing lock
    for (int i = 0; i < planes.size(); i++)
    {
      PlaneState planeState = planeStateArray[i];
      if (planeState.landingLock == true)
      {
        landingLock = true;
        break;
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
        // departure time in future or planes too close to departure location: skip
        continue;
      }

      // state clause
      if (planeState.state == PlaneState.States.NULL_STATE)
      {
        if (acquireLandingLock(planeState))
        {
          double bearing = spiralOrbit(planeState, plane.getDestination());
          bearings[i] = bearing;
        }
        else
        {
        	
          double bearing = joinOrbit(planeState, plane.getDestination(), radius);
          bearings[i] = bearing;
        }
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
          double bearing = getOrbitBearing(planeState, plane.getDestination(), radius);
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

  public boolean acquireLandingLock(PlaneState planeState)
  {
    if (landingLock == false)
    {
      landingLock = true;
      planeState.landingLock = true;
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
      landingLock = false;
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


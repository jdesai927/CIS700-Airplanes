package airplane.g5;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

import org.apache.log4j.Logger;

import airplane.sim.Plane;


public class CollisionZonePlayer extends airplane.sim.Player
{

  private Logger logger = Logger.getLogger(this.getClass()); // for logging

  // Map from plane object to plane state object
  private Map<Plane, PlaneState> planeStateMap;
  private PlaneState[] planeStateArray;

  // map from plane to zones along path it should circle
  private HashMap<Plane, ArrayList<Point2D.Double>> zoneMap;
  
  // Temporary for demo
  private static double radius = 15;
  private static boolean landingLock = false;

  private static final double WAITING = -1;
  private static final double LANDED = -2;
  private static final double MAX_TURN = 9.99999999999999;

  @Override
  public String getName()
  {
    return "Collision Zones";
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
      PlaneState planeState = new PlaneState(plane);
      planeStateMap.put(plane, planeState);
      planeStateArray[i] = planeState;
    }
    
    zoneMap = getPlaneTrajectories(planes);
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

	private static Float getSlope(Line2D line) 
	{
		Point2D.Double p1 = (Point2D.Double)line.getP1();
		Point2D.Double p2 = (Point2D.Double)line.getP2();
		
		double deltaX = p2.getX() - p1.getX();
		double deltaY = p2.getY() - p1.getY();
		
		boolean vertical = (deltaX == 0);
		Float slope = null;

		if (!vertical) 
		{
			slope = new Float(deltaY/deltaX);
		}
		
	    return slope;
	}
	
	private static boolean lineRangeIntersects(Line2D l1, Line2D l2) 
	{
		double x11 = l1.getX1();
		double x12 = l1.getX2();
		
		double x21 = l2.getX1();
		double x22 = l2.getX2();
		
		if (!(x11 < x21 && x12 <= x21) && !(x11 >= x22 && x12 > x22)) 
		{
			return true;
		}
		
		return false;
	}
	
	private static boolean lineOverlaps(Line2D l1, Line2D l2) 
	{
		Float slope1 = getSlope(l1);
		double b1 = (slope1 != null) ? l1.getY1() - slope1 * l1.getX1() : 0;
		
		Float slope2 = getSlope(l2);
		double b2 = (slope2 != null) ? l2.getY1() - slope2 * l2.getX1() : 0;
		
		if (slope1 == null && slope2 == null && lineRangeIntersects(l1, l2)) 
		{
			return true;
		}
		
		else if (slope1 != null && slope2 != null && Math.abs(slope1 - slope2) < 0.001 && 
				Math.abs(b1 - b2) < 0 && lineRangeIntersects(l1, l2)) 
		{
			return true;
		}
		
		return false;
	}

private static Point2D.Double lineIntersect(Line2D l1, Line2D l2)
{
		double denom = (l2.getY2() - l2.getY1()) * (l1.getX2() - l1.getX1()) - (l2.getX2() - l2.getX1()) * (l1.getY2() - l1.getY1());
		//lines are parallel
		if(denom == 0.0d)
			return null;
		double ua = ((l2.getX2() - l2.getX1()) * (l1.getY1() - l2.getY1()) - (l2.getY2() - l2.getY1()) * (l1.getX1() - l2.getX1()))/denom;
		double ub = ((l1.getX2() - l1.getX1()) * (l1.getY1() - l2.getY1()) - (l1.getY2() - l1.getY1()) * (l1.getX1() - l2.getX1()))/denom;
		if(ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f){
			return new Point2D.Double(l1.getX1() + ua*(l1.getX2() - l1.getX1()), l1.getY1() + ub*(l1.getY2() - l1.getY1()));
		}
		return null;
}

private static boolean lineIntersectsZone(Line2D.Double line, Point2D.Double zoneCenter) {
	double r = 5;
	double dx = line.getX2() - line.getX1();
	double dy = line.getY2() - line.getY1();
	double dr = Math.sqrt(dx * dx + dy * dy);
	double determinant = line.getX1() * line.getY2() - line.getX2() * line.getY1();
	
	double discrim = Math.pow(r, 2) * Math.pow(dr, 2) - Math.pow(determinant, 2);
	boolean intersects = (discrim > 0);
	return intersects;
}

private HashMap<Plane, ArrayList<Point2D.Double>> getPlaneTrajectories(ArrayList<Plane> planes) {
	HashMap<Plane, ArrayList<Point2D.Double>> trajectories = new HashMap<Plane, ArrayList<Point2D.Double>>();
	ArrayList<Point2D.Double> collisions = getCollisionPoints(planes);
	for (Plane p : planes) 
	{
		Line2D.Double straightPath = new Line2D.Double(p.getLocation(), p.getDestination());
		ArrayList<Point2D.Double> pathZones = new ArrayList<Point2D.Double>();
		
		for (Point2D.Double collision : collisions) 
		{
			if (lineIntersectsZone(straightPath, collision)) 
			{
				pathZones.add(collision);
			}
		}
		
		trajectories.put(p, pathZones);
	}
	
	return trajectories;
}
  

private ArrayList<Point2D.Double> getCollisionPoints(ArrayList <Plane> planes) {
	ArrayList<Point2D.Double> collisions = new ArrayList<Point2D.Double>();
	
	for (int i = 0; i < planes.size(); i++) 
	{
		Plane p1 = planes.get(i);
		Line2D.Double firstPath = new Line2D.Double(p1.getLocation(), p1.getDestination());
		
		for (int j = 0; j < planes.size(); j++) 
		{
			if (i != j) 
			{
				Plane p2 = planes.get(j);
				Line2D.Double secondPath = new Line2D.Double(p2.getLocation(), p2.getDestination());
				
				// add a time delay if these paths intersect at a point at the same time
				Point2D.Double intersection = lineIntersect(firstPath, secondPath);
				
				// add a space buffer if these paths overlap completely
				if (lineOverlaps(firstPath, secondPath)) {
					collisions.add(p1.getDestination());
					collisions.add(p2.getDestination());
				}
				
				if (intersection != null) 
				{
//					double d1 = intersection.distance(p1.getLocation());
//					double d2 = intersection.distance(p2.getLocation());
//
//					int t1 = (int) Math.round(d1/p1.getVelocity() + p1.getDepartureTime());
//					int t2 = (int)Math.round(d2/p2.getVelocity() + p2.getDepartureTime());
//
//					if (Math.abs(d1 - d2) < 5 &&  Math.abs(t1 - t2) < 0.001) 
//					{
						collisions.add(intersection);
//					}
				}
				
			}
		}
	}
	
	return collisions;
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
          ArrayList<Point2D.Double> zones = zoneMap.get(plane);
          Point2D.Double nextOrbitCenter;
          
          if (zones.size() > 0) 
          {
        	  nextOrbitCenter = zones.get(0);
          }
          else 
          {
        	  nextOrbitCenter = plane.getDestination();
          }
        	
          double bearing = joinOrbit(planeState, nextOrbitCenter, radius);
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


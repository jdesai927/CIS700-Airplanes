package airplane.g5;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.awt.geom.Point2D;

import org.apache.log4j.Logger;

import airplane.sim.Plane;



public class Group5Player extends airplane.sim.Player
{

  private Logger logger = Logger.getLogger(this.getClass()); // for logging

  // Map from plane object to plane state object
  private Map<Plane, PlaneState> planeStateMap;
  private PlaneState[] planeStateArray;

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
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      PlaneState planeState = new PlaneState(plane);
      planeStateMap.put(plane, planeState);
      planeStateArray[i] = planeState;
    }

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

    // if any plane is in the air, then just keep things as-is
    /*for (Plane p : planes)
    {
      if (p.getBearing() != -1 && p.getBearing() != -2) return bearings;
    }

    // if no plane is in the air, find the one with the earliest
    // departure time and move that one in the right direction
    int minTime = 10000;
    int minIndex = 10000;
    for (int i = 0; i < planes.size(); i++)
    {
      Plane p = planes.get(i);
      if (p.getDepartureTime() < minTime && p.getBearing() == -1)
      {
        minIndex = i;
        minTime = p.getDepartureTime();
      }
    }

    // if it's not too early, then take off and head straight for the destination
    if (round >= minTime)
    {
      Plane p = planes.get(minIndex);
      bearings[minIndex] = calculateBearing(p.getLocation(), p.getDestination());
    }*/

    // initialize bearings to original bearings
    double[] bearings = new double[planes.size()];
    for (int i = 0; i < bearingArrayOrig.length; i++)
    {
      bearings[i] = bearingArrayOrig[i];
    }

    // main decision loop iterating over each plane
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      PlaneState planeState = planeStateArray[i];
      double bearingOrig = bearingArrayOrig[i];
      if (plane.getDepartureTime() > round)
      {
        // departure time in future: skip
        continue;
      }

      if (!planeState.orbitState)
      {
        planeState.orbitState = true;
        double bearing = getOrbitBearing(planeState, plane.getDestination(), 10);
        bearings[i] = bearing;
      }
      if (planeState.orbitState)
      {
        double bearing = getOrbitBearing(planeState, plane.getDestination(), 10);
        bearings[i] = bearing;
      }
    }

    return bearings;
  }

  public double addBearings(double bearingOrig, double bearingDelta)
  {
    double retval = (bearingOrig + bearingDelta) % 360;
    if (retval < 0)
    {
      retval = 360 + retval;
    }
    if (retval > 360)
    {
      retval = retval % 360;
    }
    return retval;
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

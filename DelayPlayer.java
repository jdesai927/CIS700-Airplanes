package airplane.g5;

import java.util.List;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;

import org.apache.log4j.Logger;

import airplane.sim.Plane;
import airplane.sim.SimulationResult;

public class DelayPlayer  extends airplane.sim.Player
{

  private Logger logger = Logger.getLogger(this.getClass()); // for logging

  // Map from plane object to plane state object
  private List<PlaneState> planeStateMap;
  private List<PlaneState> planeStateMapSim;
  private PlaneState[] planeStateArray;

  private ArrayList<Plane> flyingPlanes = new ArrayList<Plane>();

  private static final double WAITING = -1;
  private static final double LANDED = -2;
  private static final double MAX_TURN = 9.99999999999999;
  private static final double COLLISION_ZONE_CRITICAL_ANGLE = 10;
  private static final float COLLISION_ZONE_RADIUS = 5;
  private static final double SAFE_SIM_DIST = 7;

  @Override
  public String getName()
  {
    return "Delay Player";
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

    // refresh simulator state
    boolean upgradeToCollision = false;
    Point2D collisionAvoidTarget = new Point2D.Double(0, 0);
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
              if ((angleOfApproach <= 180 + COLLISION_ZONE_CRITICAL_ANGLE) && (angleOfApproach >= 180 - COLLISION_ZONE_CRITICAL_ANGLE) && simPlane2.getBearing() >= 0)
              {
                PlaneState planeState1 = planeStateMap.get(simPlane1.id);
                PlaneState planeState2 = planeStateMap.get(simPlane2.id);
                Plane plane1 = planeState1.plane;
                Plane plane2 = planeState2.plane;
                //if (planeState2.state == PlaneState.States.NULL_STATE)
                {
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
          }
          break;
        }
      }
    }

    else if (!result.isSuccess())
    {
      logger.info("COULDN'T DEPART BECAUSE " + reasonToString(result.getReason()));
      depart = false;
    }

    if (upgradeToCollision)
    {
      PlaneState planeState = planeStateMap.get(planeId);
      planeState.state = PlaneState.States.COLLISION_STATE;
      planeState.currentTarget = collisionAvoidTarget;
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
    planeStateArray = new PlaneState[planes.size() + 1];
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      PlaneState planeState = new PlaneState(plane);
      planeStateMap.add(plane.id, planeState);
      planeStateArray[i] = planeState;
    }
    // initialize ids to distinguish planes
    for (int i = 0; i < planes.size(); i ++)
    {
      Plane p = planes.get(i);
      p.id = i;
      planes.set(i, p);
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


  /*
   * This is used when you're running your own simulation
   */
  @Override
  protected double[] simulateUpdate(ArrayList<Plane> planes, int round, double[] bearings)
  {
    for (int i = 0; i < planes.size(); i++)
    {
      Plane plane = planes.get(i);
      planeStateMapSim.get(i).plane = plane;
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
      Plane plane = planes.get(i);
      planeStateMap.get(i).plane = plane;
      planeStateArray[i] = planeStateMap.get(i);
    }
    for (int i = 0; i < planes.size(); i++)
    {
      Plane p = planes.get(i);
      PlaneState planeState = planeStateMap.get(p.id);
      if (flyingPlanes.contains(p) && p.getBearing() != -2)
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
      }

      else if (round >= p.getDepartureTime() && p.getBearing() != -2)
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
          
          if (!flyingPlanes.contains(p))
          {
            flyingPlanes.add(p);
          }
        }
      }
      else if (p.getBearing() == -2)
      {
        flyingPlanes.remove(p);
      }
    }

    return bearings;
  }

  public boolean reachedCollisionZone(PlaneState planeState)
  {
    Plane plane = planeState.plane;
    if(plane.getLocation().distance(planeState.currentTarget) < 1)
      return true;
    else
      return false;
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

  public double joinOrbit(PlaneState planeState, Point2D.Double center, double radius)
  {
    planeState.state = PlaneState.States.ORBIT_STATE;
    double bearing = getOrbitBearing(planeState, center, radius);
    return bearing;
  }

  public double getOrbitBearing(PlaneState planeState, Point2D center, double radius)
  {
    Plane plane = planeState.plane;
    double dist = plane.getLocation().distance(center);
    double sin = radius / dist;
    if (sin > 1) // inside the circle
      return -1;
    double radians = Math.asin(sin);
    double relativeDegree = Math.toDegrees(radians);
    double referenceDegree = calculateBearing(plane.getLocation(), (Point2D.Double)center);
    double degree = addBearings(referenceDegree, -relativeDegree);
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
}

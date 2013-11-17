package airplane.g5;
import java.util.Map;
import java.util.HashMap;
import java.util.ArrayList;

import java.awt.geom.Point2D;

public class Waypoint
{
  public Waypoint(Point2D point)
  {
    this.point = point;
  }
  public void setParent(Waypoint parent)
  {
    this.parent = parent;
  }
  public Waypoint getParent()
  {
    return parent;
  }
  public void markAsOpenList()
  {
    closedList = false;
    openList = true;
  }
  public void markAsClosedList()
  {
    closedList = true;
    openList = false;
  }
  public void unmarkFromLists()
  {
    closedList = false;
    openList = false;
  }
  public void cleanup()
  {
    unmarkFromLists();
    fScore = -1;
    if (currentSourceDistance != -1)
      storedSourceDistance = currentSourceDistance;
    currentSourceDistance = -1;
  }
  public void updateFScore(double sourceDistance, Waypoint waypointTarget)
  {
    fScore = computeFScore(sourceDistance, waypointTarget);
    currentSourceDistance = sourceDistance;
  }
  public double computeFScore(double sourceDistance, Waypoint waypointTarget)
  {
    double score = sourceDistance + getDistance(this, waypointTarget);
    return score;
  }
  public static double getDistance(Waypoint waypoint1, Waypoint waypoint2)
  {
    return Point2D.distance(waypoint1.point.getX(), waypoint1.point.getY(), waypoint2.point.getX(), waypoint2.point.getY());
  }

  public Point2D point;
  public boolean openList = false;
  public boolean closedList = false;
  public Waypoint parent = null;
  public double fScore = -1;
  public double currentSourceDistance = -1;
  public double storedSourceDistance = -1;
  Map<Route, Map<Integer, Integer> > currentTrafficMap = new HashMap<Route, Map<Integer, Integer> > ();
  public int currentTraffic = 0;
}

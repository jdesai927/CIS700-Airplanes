package airplane.g5;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;

public class Route
{
  Waypoint waypoint1;
  Waypoint waypoint2;

  public Route(Waypoint wp1, Waypoint wp2)
  {
    waypoint1 = wp1;
    waypoint2 = wp2;
    Line2D routeLine = new Line2D.Double(wp1.point, wp2.point);
    lastPathDirectional = new HashMap<Integer, ArrayList<Waypoint> > ();
  }

  int currentTraffic = 0;
  Line2D routeLine; 
  ArrayList<Waypoint> lastPath;
  Map<Integer, ArrayList<Waypoint> > lastPathDirectional;
  Set<Route> currentFlowRoutes;

  // traffic direction
  public static final int FORWARD = 1;
  public static final int BACKWARD = -1;
}

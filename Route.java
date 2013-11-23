package airplane.g5;
import java.awt.geom.Line2D;

public class Route
{
  Waypoint waypoint1;
  Waypoint waypoint2;

  public Route(Waypoint wp1, Waypoint wp2)
  {
    waypoint1 = wp1;
    waypoint2 = wp2;
    Line2D routeLine = new Line2D.Double(wp1.point, wp2.point);
  }

  int currentTraffic = 0;
  Line2D routeLine; 

  // traffic direction
  public static final int FORWARD = 1;
  public static final int BACKWARD = -1;
}

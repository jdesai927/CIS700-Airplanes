package airplane.g5;

public class Route
{
  Waypoint waypoint1;
  Waypoint waypoint2;

  public Route(Waypoint wp1, Waypoint wp2)
  {
    waypoint1 = wp1;
    waypoint2 = wp2;
  }

  int currentTraffic = 0;

}

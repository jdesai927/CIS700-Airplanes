package airplane.g5;
import airplane.sim.Plane;
import java.awt.geom.Point2D;

public class Zone
{
  public Zone(Point2D c, double r)
  {
    center = c;
    radius = r;
  }

  // copy constructor
  public Zone(Zone zone)
  {
    this(zone.center, zone.radius);
    this.population = zone.population;
    this.landingLock = zone.landingLock;
  }

  Point2D center;
  double radius = 0;
  double population = 0;
  boolean landingLock = false;
  
}

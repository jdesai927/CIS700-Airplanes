package airplane.g5;
import airplane.sim.Plane;

public class PlaneState
{
  public PlaneState(Plane p)
  {
    plane = p;
  }
  Plane plane;
  boolean orbitState = false;
  boolean orbitReached = false;
  double orbitBearing = 0;
}

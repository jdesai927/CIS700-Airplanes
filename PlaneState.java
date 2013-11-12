package airplane.g5;
import airplane.sim.Plane;

public class PlaneState
{
  public PlaneState(Plane p)
  {
    plane = p;
  }
  
  Plane plane;
  Trajectory path;
  boolean orbitState = false;
  boolean orbitReached = false;
  double orbitBearing = 0;
 
  public void setPath(Trajectory t) {
	  path = t;
  }
  
}

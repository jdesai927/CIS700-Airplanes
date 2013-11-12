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

  // states
  public enum States
  {
    NULL_STATE, ORBIT_STATE, SPIRAL_STATE;
  }

  States state = States.NULL_STATE;

  double bearingOrig = 0;
  boolean landingLock = false;
 
  public void setPath(Trajectory t) {
	  path = t;
  }
  
}

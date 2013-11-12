package airplane.g5;
import airplane.sim.Plane;

public class PlaneState
{
  public PlaneState(Plane p)
  {
    plane = p;
  }

  // copy constructor
  public PlaneState(PlaneState planeState)
  {
      this(planeState.plane);
      if (planeState.path != null)
        this.path = new Trajectory(planeState.path);
      this.state = planeState.state;
      this.bearingOrig = planeState.bearingOrig;
      this.landingLock = planeState.landingLock;
      if (planeState.currentZone != null)
        this.currentZone = new Zone(planeState.currentZone);
      if (planeState.simulationTrajectory != null)
        this.simulationTrajectory = new Trajectory(planeState.simulationTrajectory);
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
  Zone currentZone;
  Trajectory simulationTrajectory;
 
  public void setPath(Trajectory t) {
	  path = t;
  }
  
}

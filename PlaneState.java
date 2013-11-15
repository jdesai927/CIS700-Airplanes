package airplane.g5;
import java.util.ArrayList;
import airplane.g5.Trajectory;
import airplane.sim.Plane;
import java.awt.geom.Point2D;

public class PlaneState
{
 Plane plane;
 Trajectory path;
	  
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
      if (planeState.currentTarget != null)
        this.currentTarget = new Point2D.Double(planeState.currentTarget.getX(), planeState.currentTarget.getY());
      if (planeState.currentZone != null)
        this.currentZone = new Zone(planeState.currentZone);
      if (planeState.simulationTrajectory != null)
        this.simulationTrajectory = new Trajectory(planeState.simulationTrajectory);
  }
  


  // states
  public enum States
  {
    NULL_STATE, ORBIT_STATE, SPIRAL_STATE, COLLISION_STATE;
  }

  States state = States.NULL_STATE;

  double bearingOrig = 0;
  boolean landingLock = false;
  Zone currentZone;
  Point2D currentTarget;
  Trajectory simulationTrajectory;
 
  public void setPath(Trajectory t) {
	  path = t;
  }
  
}

package airplane.g5;

import java.awt.Shape;

public class Trajectory {
	
	private Shape path;
	
  // states
  public enum TrajectoryType
  {
    LINE, ORBIT;
  }
  TrajectoryType trajectoryType;

	public Trajectory(Shape s) {
		path = s;
	}

  // copy constructor
  public Trajectory(Trajectory trajectory)
  {
    this(trajectory.getPath());
    this.trajectoryType = trajectory.trajectoryType;
  }
	
	public Shape getPath() {
		return path;
	}
	
}

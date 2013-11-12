package airplane.g5;

import java.awt.Shape;

public class Trajectory {
	
	private Shape path;
	
	public Trajectory(Shape s) {
		path = s;
	}
	
	public Shape getPath() {
		return path;
	}
	
}

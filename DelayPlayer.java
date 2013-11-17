package airplane.g5;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;

import org.apache.log4j.Logger;

import airplane.sim.Plane;
import airplane.sim.SimulationResult;

public class DelayPlayer  extends airplane.sim.Player {

	private Logger logger = Logger.getLogger(this.getClass()); // for logging
	private HashSet<Integer> departedPlanes = new HashSet<Integer>();
	private ArrayList<Plane> flyingPlanes = new ArrayList<Plane>();
	
	@Override
	public String getName() {
		return "Delay Player";
	}
	
	private String reasonToString(int reason) {
		String reasonString = "";
		switch(reason) {
		case SimulationResult.ILLEGAL_BEARING:
			reasonString = "illegal bearing";
			break;
		case SimulationResult.NORMAL:
			reasonString = "normal";
			break;
		case SimulationResult.NULL_BEARINGS:
			reasonString = "null bearings";
			break;
		case SimulationResult.OUT_OF_BOUNDS:
			reasonString = "out of bounds";
			break;
		case SimulationResult.STOPPED:
			reasonString = "stopped";
			break;
		case SimulationResult.TOO_CLOSE:
			reasonString = "too close";
			break;
		case SimulationResult.TOO_EARLY:
			reasonString = "too early";
			break;
		}
	
		return reasonString;
	}
	
	private boolean depart(int planeId, int round, ArrayList<Plane> planes) {
		boolean depart = true;
		ArrayList<Plane> planesToSim = new ArrayList<Plane>();
		planesToSim.addAll(flyingPlanes);
		planesToSim.add(planes.get(planeId));
		
		SimulationResult result = startSimulation(planesToSim, round);
		
		if (result.getReason() == SimulationResult.TOO_CLOSE) {
			ArrayList<Plane> simPlanes = result.getPlanes();
			// find which planes are too close
			for (Plane plane1: simPlanes) {
				Point2D.Double p1 = plane1.getLocation(); 
				for (Plane plane2: simPlanes) {
					Point2D.Double p2 = plane2.getLocation();
					if (plane1.id != plane2.id && p1.distance(p2) <= 5) {
						
						// the current plane will collide with a currently flying plane if it departs
						if (plane1.id == planeId || plane2.id == planeId) {
							return false;
						}
					}
				}
			}
		}

		else if (!result.isSuccess()) {
			logger.info("COULDN'T DEPART BECAUSE " + reasonToString(result.getReason()));
			depart = false;
		}
		
		return depart;
	}
	
	
	/*
	 * This is called at the beginning of a new simulation. 
	 * Each Plane object includes its current location (origin), destination, and
	 * current bearing, which is -1 to indicate that it's on the ground.
	 */
	@Override
	public void startNewGame(ArrayList<Plane> planes) {
		logger.info("Starting new game!");
		
		// initialize ids to distinguish planes
		for (int i = 0; i < planes.size(); i ++) {
			Plane p = planes.get(i);
			p.id = i;
			planes.set(i, p);
		}
	}
	
	
private Comparator<Plane> INCREASING_DEPT = new Comparator<Plane>(){
	public int compare(Plane p1, Plane p2) {
		if (p2.getDepartureTime() < p1.getDepartureTime()) {
			return 1;
		}
		else if (p1.getDepartureTime() < p2.getDepartureTime()) {
			return -1;
		}
		else {
			return 0;
		}
	}
};

private Comparator<Plane> DECREASING_COST = new Comparator<Plane>(){
	public int compare(Plane p1, Plane p2) {
		double cost1 = p1.getDepartureTime() + p1.getLocation().distance(p1.getDestination());
		double cost2 = p2.getDepartureTime() + p2.getLocation().distance(p2.getDestination());
		if (cost1 < cost2) {
			return 1;
		}
		else if (cost2 < cost1) {
			return -1;
		}
		else {
			return 0;
		}
	}
};

	
private void dispatchNextPlane() {
//	if (departureIds.size() > 1) {
//		departureIds.remove(0);
//		flyingPlane = departureIds.get(0);
//	}
}

/*
 * This is used when you're running your own simulation
 */
  @Override
  protected double[] simulateUpdate(ArrayList<Plane> planes, int round, double[] bearings) {
		for (int i = 0; i < planes.size(); i++) {
			Plane p = planes.get(i);
			if (round >= p.getDepartureTime() && p.getBearing() != -2) {
				bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
			}
		}
		return bearings;
	}
	
	/*
	 * This is called at each step of the simulation.
	 * The List of Planes represents their current location, destination, and current
	 * bearing; the bearings array just puts these all into one spot. 
	 * This method should return an updated array of bearings.
	 */
	@Override
	public double[] updatePlanes(ArrayList<Plane> planes, int round, double[] bearings) {		
		for (int i = 0; i < planes.size(); i++) {
			Plane p = planes.get(i);
			if (departedPlanes.contains(p.id) && p.getBearing() != -2) {
				bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
			}
			
			else if (round >= p.getDepartureTime()) {
				if ((depart(p.id, round, planes)) && p.getBearing() != -2) {
					bearings[i] = calculateBearing(p.getLocation(), p.getDestination());
					departedPlanes.add(i);
					
					if (!flyingPlanes.contains(p)) {
						flyingPlanes.add(p);
					}
				}
			}
		}
		
		return bearings;
	}
	
}

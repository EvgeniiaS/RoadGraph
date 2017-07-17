package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

/**
 * Class represents a geographic point on the MapGraph.
 * Contains information about adjacent geographic points, the distance
 * to reach this point, and the distance from the goal point on the map 
 * to this point.
 * @author Evgeniia Shcherbinina
 *
 */
public class MapNode implements Comparable<MapNode> {
	
	// default distance in km to reach this point
	static final double MAX_DISTANCE = 50000;
	
	
	// geographic location on the map
	private GeographicPoint point;
	
	// a list of adjacent points
	private List<MapEdge> adjacentEdges;	
	
	// the distance in km to reach this point
	private double distance;
	
	// the distance in km between this point and the goal
	private double distanceToGoal;

	/**
	 * Constructs a new MapNode object.
	 * @param point the geographic location of this point
	 */
	public MapNode(GeographicPoint point) {
		this.point = point;
		adjacentEdges = new LinkedList<>();
		distance = MAX_DISTANCE;
		distanceToGoal = 0;
	}
	
	/**
	 * Returns point.
	 * @return point the geographic point
	 */
	public GeographicPoint getPoint() {
		return point;
	}
	
	/**
	 * Adds an outgoing edge from this point.
	 * @param end ending point of the edge
	 * @param name name of the street between two points
	 * @param distance between two points in km
	 */
	public void addEdge(GeographicPoint end, String name, String roadType, double distance) {
		MapEdge edge = new MapEdge(point, end, name, roadType, distance);
		adjacentEdges.add(edge);
	}
	
	/**
	 * Gets all out-neighbors of the geographic point.
	 * @return a list of geographic points
	 */
	public List<GeographicPoint> getNeighbors() {
		List<GeographicPoint> neighbors = new LinkedList<>();
		for(MapEdge e: adjacentEdges) {
			neighbors.add(e.getEndPoint());
		}
		return neighbors;
	}
	
	/**
	 * Gets the distance from this point to a neighbor.
	 * @param neighbor the geographic point which is adjacent to this point
	 * @return the distance in km between this point and its neighbor
	 */
	public double getDistanceToNeighbor(MapNode neighbor) {
		for (MapEdge edge: adjacentEdges) {
			if (edge.getEndPoint().equals(neighbor.getPoint())) {
				return edge.getLength();
			}
		}
		// the distance is unknown
		return MAX_DISTANCE;
	}
	
	/**
	 * Gets the distance to reach this point.
	 * @return the distance in km
	 */
	public double getDistance() {
		return distance;
	}
	
	/**
	 * Sets the distance to reach this point.
	 * Throws IllegalArgumentException if the parameter is less than 0.
	 * @param distance the distance to reach this point
	 */
	public void setDistance(double distance) {
		if (distance < 0) {
			throw new IllegalArgumentException();
		}
		this.distance = distance;
	}
	
	/**
	 * Gets the distance from this point to the goal point.
	 * @return the distance in km to the goal point
	 */
	public double getDistanceToGoal() {
		return distanceToGoal;
	}
	
	/**
	 * Sets the distance from this point to the goal point.
	 * Throws IllegalArgumentException if the parameter is less than 0.
	 * @param d the distance to be set
	 */
	public void setDisatnceToGoal(double d) {
		if (distance < 0) {
			throw new IllegalArgumentException();
		}
		distanceToGoal = d;
	}
	
	/**
	 * Returns the number of adjacent edges.
	 * @return the number of edges
	 */
	public int size() {
		return adjacentEdges.size();
	}

	@Override
	public int compareTo(MapNode node) {
		if (this.distance + this.distanceToGoal < node.distance + node.distanceToGoal) {
			return -1; // higher priority
		} else if (this.distance + this.distanceToGoal > node.distance + node.distanceToGoal) {
			return 1; // lower priority
		}
		return 0; // equal priority
	}
	
	/** ToString to print out a MapNode object
	 *  @return the string representation of a MapNode
	 */
	@Override
	public String toString()
	{
		String toReturn = "[NODE at location (" + point + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: adjacentEdges) {
			toReturn += e.getStreetName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}

	// For debugging, output roadNames as a String.
	public String roadNamesAsString()
	{
		String toReturn = "(";
		for (MapEdge e: adjacentEdges) {
			toReturn += e.getStreetName() + ", ";
		}
		toReturn += ")";
		return toReturn;
	}

}

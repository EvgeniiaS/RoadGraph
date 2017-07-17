package roadgraph;

import geography.GeographicPoint;

/**
 * Class represents the edge between two geographic points on the map.
 * @author Evgeniia Shcherbinina
 */
public class MapEdge {

	// Starting intersection location
	private GeographicPoint start;
	
	// Ending intersection location
	private GeographicPoint end;
	
	// Street between two intersections
	private String streetName;

	// Type of the road between two points
	private String roadType;
		
	// Distance in km between two intersections
	private double length;
	
	 
	/**
	 * Constructs a new object MapEdge
	 * @param point1 the start point
	 * @param point2 the end point
	 * @param name the name of the street between points
	 * @param roadType the road type between points
	 * @param distance the distance in km between two points
	 */
	public MapEdge(GeographicPoint point1, GeographicPoint point2, String name, 
			String roadType, double distance) {
		start = point1;
		end = point2;
		streetName = name;
		this.roadType = roadType;
		length = distance;
	}
	
	/**
	 * Returns starting intersection location.
	 * @return starting intersection location
	 */
	public GeographicPoint getStartPoint() {
		return start;
	}
	
	/**
	 * Returns ending intersection location.
	 * @return ending intersection location
	 */
	public GeographicPoint getEndPoint() {
		return end;
	}
	
	/**
	 * Returns street name between two intersections.
	 * @return street name
	 */
	public String getStreetName() {
		return streetName;
	}
	
	/**
	 * Returns the type of the road between two intersections.
	 * @return the type of the road
	 */
	public String getRoadType() {
		return roadType;
	}

	/**
	 * Returns the distance between two intersections.
	 * @return length
	 */
	public double getLength() {
		return length;
	}
	
}


package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;

/** 
 * Class represents a graph of geographic locations.
 * Nodes in the graph are intersections between 
 * @author Evgeniia Shcherbinina
 */
public class MapGraph {
	
	// the maximum distance between two geographic points
	static final double MAX_DISTANCE = 50000;
	
	HashMap<GeographicPoint, MapNode> map;
	
	
	/** 
	 * Constructs a new empty MapGraph 
	 */
	public MapGraph() {
		map = new HashMap<>();
	}
	
	/**
	 * Returns the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return map.size();
	}
	
	/**
	 * Returns the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return map.keySet();
	}
	
	/**
	 * Returns the number of road segments in the graph.
	 * @return The number of edges in the graph
	 */
	public int getNumEdges() 	{
		int numberEdges = 0;
		Set<GeographicPoint> setPoints = getVertices();
		for (GeographicPoint point: setPoints) {
			MapNode node = map.get(point);
			numberEdges += node.size();
		}
		return numberEdges;
	}

	
	
	/** Adds a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (map.containsKey(location) || location == null) {
			return false;
		}
		map.put(location, new MapNode(location));
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if (!map.containsKey(from) || !map.containsKey(to) || roadName == null || roadType == null
				|| length < 0) {
			throw new IllegalArgumentException();
		} else {
			map.get(from).addEdge(to, roadName, roadType, length);
		}
	}
	
	
	/** 
	 * Finds the path from start to goal using breadth first search
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		// check input
		if (start == null || goal == null) {
			throw new NullPointerException();
		}
		if (!map.containsKey(start)) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (!map.containsKey(goal)) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}
		
		// initialization
		Queue<GeographicPoint> explore = new LinkedList<>();
		HashSet<GeographicPoint> visited = new HashSet<>();	
		HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<>();
		
		explore.add(start);
		visited.add(start);
		
		findPath(explore, visited, parents, nodeSearched, goal);
		return pathRecover(parents, start, goal);
	}
	
	/**
	 * A helper method to find the path between two points.
	 * @param explore a list of points to explore
	 * @param visited a list of visited points
	 * @param nodeSearched for visualization
	 * @return a map of connections between points
	 */
	private void findPath(Queue<GeographicPoint> explore, HashSet<GeographicPoint> visited,
			HashMap<GeographicPoint, GeographicPoint> parents, Consumer<GeographicPoint> nodeSearched,
			GeographicPoint goal) {
		
		while(!explore.isEmpty()) {
			GeographicPoint current = explore.remove();
			
			//for visualization
			nodeSearched.accept(current);
			
			if (current.equals(goal)) {
				break;
			}
			
			List<GeographicPoint> neighbors = map.get(current).getNeighbors();
			for (GeographicPoint p: neighbors) {
				if(!visited.contains(p)) {
					explore.add(p);
					visited.add(p);
					parents.put(p, current);
				}
			}
		}
	}
	
	/**
	 * Recovers the path from start to the end point.
	 * @param parents keeps information about relations between points
	 * @param start start point
	 * @param goal end point
	 * @return a list of points from start to end (including start and end)
	 */
	private List<GeographicPoint> pathRecover(HashMap<GeographicPoint, GeographicPoint> parents,
			GeographicPoint start, GeographicPoint goal) {
		List<GeographicPoint> path = new LinkedList<>();
		GeographicPoint next = goal;
		while (!next.equals(start)) {
			path.add(0, next);
			next = parents.get(next);
		}
		path.add(0, next);
		return path;
	}
	
	
	/** 
	 * Finds the path from start to goal using Dijkstra's algorithm.
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
					           			  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		// check input
		if (start == null || goal == null) {
			throw new NullPointerException();
		}
		
		MapNode first = map.get(start);		
		MapNode last = map.get(goal);
		
		if (first == null) {
			System.err.println("Start node does not exist");
			return null;
		}
		if (last == null) {
			System.err.println("End node does not exist");
			return null;
		}			
			
		// initializing
		PriorityQueue<MapNode> explore = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<>();
		
		// initialize the distance of all vertices to 50000
		Set<GeographicPoint> vertices = getVertices();
		for (GeographicPoint v: vertices) {
			MapNode node = map.get(v);
			node.setDisatnceToGoal(0);
			node.setDistance(MAX_DISTANCE);
		}
		
		first.setDistance(0);		
		explore.add(first);
		
		HashMap<GeographicPoint, GeographicPoint> parents = findPathD(explore, visited, nodeSearched,
																	 first, last);
		
		if (parents != null) {
			return pathRecover(parents, start, goal);
		}
		return null;
	}
	
	/**
	 * A helper method to find the path between two points using Dijkstra's algorithm.
	 * @param explore a list of nodes to explore
	 * @param visited a set of visited nodes
	 * @param nodeSearched for visualization
	 * @param start the starting point
	 * @param goal the end point
	 * @return a map of connections between points
	 */
	private HashMap<GeographicPoint, GeographicPoint> findPathD(PriorityQueue<MapNode> explore,
			HashSet<MapNode> visited, Consumer<GeographicPoint> nodeSearched,
			MapNode first, MapNode last) {
		
		HashMap<GeographicPoint, GeographicPoint> parents = new HashMap<>();
		while (!explore.isEmpty()) {			
			MapNode current = explore.remove();
			
			//for visualization
			nodeSearched.accept(current.getPoint());
			
			if (!visited.contains(current)) {
				visited.add(current);
				if(current.equals(last)) {
					return parents;
				}
				List<GeographicPoint> neighbors = current.getNeighbors();
				
				for (GeographicPoint p: neighbors) {
					MapNode pNode = map.get(p);
					if (!visited.contains(pNode)) {
						double distance = current.getDistance() + current.getDistanceToNeighbor(pNode);
						if (distance < pNode.getDistance()) {
							pNode.setDistance(distance);
							parents.put(p, current.getPoint());
							explore.add(pNode);
						}
					}
				}
			}
		}	
		// the path between two points doesn't exist
		return null;
	}

	
	/** 
	 * Finds the path from start to goal using A-Star search.
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		// check input
		if (start == null || goal == null) {
			throw new NullPointerException();
		}
				
		MapNode first = map.get(start);		
		MapNode last = map.get(goal);
				
		if (first == null) {
			System.err.println("Start node does not exist");
			return null;
		}
		if (last == null) {
			System.err.println("End node does not exist");
			return null;
		}				
					
		// initializing
		PriorityQueue<MapNode> explore = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<>();
				
		// initialize the distance of all vertices to 50000
		// and distances between the start point and all vertices
		Set<GeographicPoint> vertices = getVertices();
		for (GeographicPoint v: vertices) {
			MapNode node = map.get(v);
			node.setDistance(MAX_DISTANCE);
			node.setDisatnceToGoal(v.distance(goal));
		}
				
		first.setDistance(0);		
		explore.add(first);
				
		HashMap<GeographicPoint, GeographicPoint> parents = findPathD(explore, visited, nodeSearched,
						 first, last);

		if (parents != null) {
			return pathRecover(parents, start, goal);
		}
		// the path between two points doesn't exist
		return null;					
	}
	
}

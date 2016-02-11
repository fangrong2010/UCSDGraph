/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Set;
import java.util.HashSet;
import java.util.Map;
import java.util.HashMap;
import java.util.PriorityQueue;
import java.util.Collection;
import java.util.Comparator;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;


/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	
	Map<GeographicPoint, MapVertex> Vertices;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		
		Vertices = new HashMap<GeographicPoint, MapVertex>();
		
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		
		return Vertices.size();
//		return 0;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		
		return Vertices.keySet(); // return the set view of the keys in Vertices
//		return new HashSet<GeographicPoint>(Vertices.keySet());

	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		
		int NumEdges = 0;
		Collection<MapVertex> vertices = Vertices.values();
		for(MapVertex vertex : vertices){ // accumulate each vertex's number of connected edges
			NumEdges += vertex.getSize();
		}

		return NumEdges;
//		return 0;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		
		if(location == null || Vertices.containsKey(location))
			return false;
		
		Vertices.put(new GeographicPoint(location.x, location.y), new MapVertex(location));
		
		return true;
		
//		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
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

		//TODO: Implement this method in WEEK 2
		if(!Vertices.containsKey(from) || !Vertices.containsKey(to) ||
			from == null || to == null || roadName == null || roadType == null || length < 0){
			throw new IllegalArgumentException("arguments are not valid");
		}
		
		MapEdge edge = new MapEdge(Vertices.get(from), Vertices.get(to), roadName, roadType, length);
		MapVertex vertex = Vertices.get(from); // find the associated vertex
		vertex.ConnectTo(edge); // add the newly created edge to vertex
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		queue.add(start);
		visited.add(start);
		while(!queue.isEmpty()){
			GeographicPoint curr = queue.poll();
			nodeSearched.accept(curr);
			if(curr.equals(goal)){ // reaches the goal
				return RetrievePath(parentMap, start, goal); // retrieve the path
			}
			MapVertex node = Vertices.get(curr); // the MapVertex associated with the location curr
			Set<MapEdge> edges = node.getEdges(); // the edges connected to the node
			for(MapEdge edge : edges){ 
				MapVertex next = edge.getEnd(); // neighbor of the location curr
				GeographicPoint nextLoc = next.getLocation();
				if(!visited.contains(nextLoc)){
					visited.add(nextLoc);
					parentMap.put(nextLoc, curr); // set curr as the parent of next in tha path map
					queue.add(nextLoc);
				}
			}
		}
		/////// if the goal is not achievable
		return null;
	}
	
	private List<GeographicPoint> RetrievePath(Map<GeographicPoint, GeographicPoint> parentMap, 
			GeographicPoint start, GeographicPoint goal){
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		GeographicPoint curr = goal;
		while(!curr.equals(start)){
			path.add(0, curr);
			curr = parentMap.get(curr);
		}
		path.add(0, curr);
		
		return path;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		PriorityQueue<MapVertex> pq = new PriorityQueue<MapVertex>();
		Set<MapVertex> visited = new HashSet<MapVertex>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		setDistancesFromStart(start);
		pq.add(Vertices.get(start)); // put the start node into the PQ
		while(!pq.isEmpty()){
			MapVertex curr = pq.poll();
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)){
				visited.add(curr);
				if(curr.getLocation().equals(goal))
					return RetrievePath(parentMap, start, goal);; // retrieve the shortest path
				Set<MapEdge> edges = curr.getEdges();
				for(MapEdge edge : edges){
					MapVertex next = edge.getEnd();
					if(!visited.contains(next)){
						double currLen = curr.getDistFromStart() + edge.getRoadLength();
						if(currLen < next.getDistFromStart()){
							next.setDistFromStart(currLen); //update shortest distance to the start
							parentMap.put(next.getLocation(), curr.getLocation()); // update the parent-child relation
							pq.add(next);
						}
					}
				}
			}
		}
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		PriorityQueue<MapVertex> pq = new PriorityQueue<MapVertex>(MapVertex.AStarComparator);
		Set<MapVertex> visited = new HashSet<MapVertex>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		setDistancesFromStart(start);
		setDistancesToEnd(start, goal);
		pq.add(Vertices.get(start)); // put the start node into the PQ
		while(!pq.isEmpty()){
			MapVertex curr = pq.poll();
			nodeSearched.accept(curr.getLocation()); // for visualization
			if(!visited.contains(curr)){
				visited.add(curr);
				if(curr.getLocation().equals(goal))
					return RetrievePath(parentMap, start, goal);; // retrieve the shortest path
				Set<MapEdge> edges = curr.getEdges();
				for(MapEdge edge : edges){
					MapVertex next = edge.getEnd();
					if(!visited.contains(next)){
						double currLenFromStart = curr.getDistFromStart() + edge.getRoadLength();
						if(currLenFromStart < next.getDistFromStart()){
							next.setDistFromStart(currLenFromStart); //update shortest distance to the start
							parentMap.put(next.getLocation(), curr.getLocation()); // update the parent-child relation
							pq.add(next);
						}
					}
				}
			}
		}
		
		return null;
	}
	
	/*
	 * some helper functions
	 */
	
	/**
	 * Set the distances from each vertex to the starting point as appropriate
	 */
	private void setDistancesFromStart(GeographicPoint start){
		Collection<MapVertex> allNodes = Vertices.values();
		for(MapVertex node : allNodes){
			node.setDistFromStart(Double.POSITIVE_INFINITY);
		}
		MapVertex source = Vertices.get(start); 
		source.setDistFromStart(0.0); // start to itself is of 0 distance
	}
	
	/**
	 * initialize the predicted distances from each node to the goal as appropriate
	 */
	private void setDistancesToEnd(GeographicPoint start, GeographicPoint goal){
		Collection<MapVertex> allNodes = Vertices.values();
		for(MapVertex node : allNodes){
//			node.setPredictedDistToEnd(Double.POSITIVE_INFINITY);
			node.setPredictedDistToEnd(calculateDistance(node, Vertices.get(goal)));
		}
//		MapVertex destination = Vertices.get(goal);
//		destination.setPredictedDistToEnd(0.0);
//		MapVertex source = Vertices.get(start); // set the predicted dist from start to goal as the straight line dist
//		source.setPredictedDistToEnd(calculateDistance(source, destination));
	}
	
	/**
	 * @return the distance b/w two points given their latitude and longitude
	 */
	private double calculateDistance(MapVertex source, MapVertex destination){
		double EarthRadius = 6371000.0;
		GeographicPoint sourceLocation = source.getLocation();
		GeographicPoint destinationLocation = destination.getLocation();
		double sourceLatitude = Math.toRadians(sourceLocation.x);
		double sourceLongitude = Math.toRadians(sourceLocation.y);
		double destinationLatitude = Math.toRadians(destinationLocation.x);
		double destinationLongitude = Math.toRadians(destinationLocation.y);
		double latDiff = sourceLatitude - destinationLatitude;
		double lonDiff = sourceLongitude - destinationLongitude;
		double a = Math.sin(latDiff/2) * Math.sin(latDiff/2) + 
				   Math.cos(sourceLatitude) * Math.cos(destinationLatitude) * Math.sin(lonDiff/2) * Math.sin(lonDiff/2);
		double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
		
		return EarthRadius * c;
	}
	
	/**
	 * print the graph
	 * @return
	 */
	public String toString(){
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		/* Use this code in Week 2
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		*/
		
		// You can use this method for testing. 
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");

		GraphLoader.loadRoadMap("data/graders/mod3/map2.txt", theMap);
		System.out.println("DONE.");

		System.out.println("Num nodes: " + theMap.getNumVertices());
		System.out.println("Num edges: " + theMap.getNumEdges());
		
		List<GeographicPoint> route = theMap.dijkstra(new GeographicPoint(1.0,1.0), 
												 new GeographicPoint(8.0,-1.0));
		
		System.out.println(route);
		
		List<GeographicPoint> route2 = theMap.aStarSearch(new GeographicPoint(1.0,1.0), 
												 new GeographicPoint(8.0,-1.0));
		System.out.println(route2);
		
//		System.out.println(Double.POSITIVE_INFINITY + Double.POSITIVE_INFINITY);
		
		/* Use this code in Week 3 End of Week Quiz
		
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}

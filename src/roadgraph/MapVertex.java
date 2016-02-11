package roadgraph;

import java.util.List;
import java.util.Set;
import java.util.HashSet;
import geography.GeographicPoint;
import java.util.ArrayList;

/**
 * a class to represent a vertex in a graph
 * @author fangrong
 * @param location of the vertex
 * @param list of edges that starts from the vertex
 */

public class MapVertex implements Comparable<MapVertex>{
	
	private GeographicPoint location;
//	private List<MapEdge> edges;
	private Set<MapEdge> edges;
	// for Week3
	private double distFromStart;
	private double predictedDistToEnd;
	
	/**
	 * create a MapVertex object given a location
	 */
	public MapVertex(GeographicPoint location){
		this.location = new GeographicPoint(location.x, location.y);
//		edges = new ArrayList<MapEdge>();
		edges = new HashSet<MapEdge>();
		distFromStart = 0.0;
		predictedDistToEnd = 0.0;
	}
	
	/**
	 * add an edge starting from the vertex in edges
	 * @param edge is the edge to be added
	 */
	public void ConnectTo(MapEdge edge){
//		if(!edges.contains(edge) && edge != null)
			edges.add(edge);
	}
	
	/**
	 * get the location of the vertex
	 */
	public GeographicPoint getLocation(){
		return new GeographicPoint(this.location.x, this.location.y);
	}
	
	/**
	 * get the list of edges
	 * @return
	 */
//	public List<MapEdge> getEdges(){
//		return new ArrayList<MapEdge>(edges);		
//	}
	public Set<MapEdge> getEdges(){
		return new HashSet<MapEdge>(edges);
	}
	
	/**
	 * get the number of edges that the vertex is connected to
	 * @return the number of edges
	 */
	public int getSize(){
		return edges.size();
	}
	
	/**
	 * get the distance from the point to the start point 
	 * @return the distance b/w the point and the start
	 */
	public double getDistFromStart(){
		return distFromStart;
	}
	/**
	 * set the distance from the point to the start point
	 */
	public void setDistFromStart(double dist){
		this.distFromStart = dist;
	}
	
	/**
	 * get the predicted distance from the point to the end point 
	 * @return the predicted distance b/w the point and the end
	 */
	public double getPredictedDistToEnd(){
		return predictedDistToEnd;
	}
	/**
	 * set the predicted distance from the point to the end
	 */
	public void setPredictedDistToEnd(double dist){
		this.predictedDistToEnd = dist;
	}
	
	/*
	 * compareTo() implements Comparable
	 */
	public int compareTo(MapVertex other){
		if(this.getDistFromStart() < other.getDistFromStart())
			return -1;
		else if(this.getDistFromStart() > other.getDistFromStart())
			return 1;
		else
			return 0;
	}
}

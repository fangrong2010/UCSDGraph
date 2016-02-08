package roadgraph;

import java.util.List;

import geography.GeographicPoint;

import java.util.ArrayList;

/**
 * a class to represent a vertex in a graph
 * @author fangrong
 * @param location of the vertex
 * @param list of edges that starts from the vertex
 */

public class MapVertex {
	
	private GeographicPoint location;
	private List<MapEdge> edges;
	
	/**
	 * create a MapVertex object given a location
	 */
	public MapVertex(GeographicPoint location){
		this.location = new GeographicPoint(location.x, location.y);
		edges = new ArrayList<MapEdge>();
	}
	
	/**
	 * add an edge starting from the vertex in edges
	 * @param edge is the edge to be added
	 */
	public void ConnectTo(MapEdge edge){
		if(!edges.contains(edge) && edge != null)
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
	public List<MapEdge> getEdges(){
		return new ArrayList<MapEdge>(edges);		
	}
	
	/**
	 * get the number of edges that the vertex is connected to
	 * @return
	 */
	public int getSize(){
		return edges.size();
	}
}

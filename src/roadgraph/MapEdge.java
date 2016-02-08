package roadgraph;

import geography.GeographicPoint;

/**
 * a class which represents a graph edge
 * @param location of the starting point
 * @param location of the ending point
 * @param street name
 * @param street type
 * @param street length
 * @author Fangrong Peng
 *
 */
public class MapEdge {
	
	private GeographicPoint start;
	private GeographicPoint end;
	private String roadName;
	private String roadType;
	private double roadLength;
	
	/**
	 *  create a new MapEdge object
	 */
	public MapEdge(GeographicPoint source, GeographicPoint destination, String Name, String Type, double Len){
		this.start = new GeographicPoint(source.x, source.y);
		this.end = new GeographicPoint(destination.x, destination.y);
		this.roadName = new String(Name);
		this.roadType = new String(Type);
		this.roadLength = Len;
	}
	
	/**
	 * get the start point
	 * @return the start point of the edge
	 */
	public GeographicPoint getStart(){
		return new GeographicPoint(start.x, start.y);
	}
	
	/**
	 * get the end point
	 * @return the end point of the edge
	 */
	public GeographicPoint getEnd(){
		return new GeographicPoint(end.x, end.y);
	}
	
	/**
	 * get the road name
	 * @return the name of the road
	 */
	public String getRoadName(){
		return new String(this.roadName);
	}
	
	/**
	 * get the type of the road
	 * @return the type of the road
	 */
	public String getRoadType(){
		return new String(this.roadType);
	}
	
	/**
	 * get the length of the road
	 * @return the length of the road
	 */
	public double getRoadLength(){
		return this.roadLength;
	}
}

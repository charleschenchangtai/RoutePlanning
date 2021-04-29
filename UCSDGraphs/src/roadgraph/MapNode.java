package roadgraph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

public class MapNode implements Comparable {
	//The list of edges out of the node
	private HashSet<MapEdge> edges;
	//The latitude and longitude of this node
	private GeographicPoint location;
	//The predicted distance of this node
	private double distance;
	//The actual distance of this node from start
	private double actualDistance;

	//Constructor
	public MapNode(GeographicPoint loc) {
		// TODO Auto-generated constructor stub
		location = loc;
		edges = new HashSet<MapEdge>();
		distance = 0.0;
		actualDistance = 0.0;
	}
	public void addEdge(MapEdge edge){
		edges.add(edge);
	}
	//Return the neighbors of this MapNode
	public Set<MapNode> getNeighbors(){
	    Set<MapNode> neighbors = new HashSet<MapNode>();
	    for(MapEdge edge :edges) {
	    	neighbors.add(edge.getOtherNode(this));
	    }
		return neighbors;
	}
	//get the location of a node
	public GeographicPoint getLocation(){
		return location;
	}
	
	//Return the edges out of this node 
	Set<MapEdge> getEdges(){
		return edges;
	}
	
	@Override
	public int compareTo(Object o) {
		MapNode m = (MapNode)o; 
		return ((Double)this.getDistance()).compareTo((Double) m.getDistance());
	}
	//get node distance (predicted)
	public Double getDistance() {
		// TODO Auto-generated method stub
		return this.distance;
	}
	//set node distance (predicted)
	public void setDistance(double distance) {
		this.distance = distance;
	}
	//get node distance (actual)
	public double getActualDistance() {
		return this.actualDistance;
	}
	// set node distance (actual)	
	public void setActualDistance(double actualDistance) {
		this.actualDistance = actualDistance;
	}
	
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	public String toString()
	{
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}
	
	
	
	
	
	
}

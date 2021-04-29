package roadgraph;

public class MapEdge {
	// Locations of start and end points 
	private MapNode start;
	private MapNode end;
	
	//The name of the road
	private String roadName;
	
	//The type of the road
	private String roadType;
	
	//The length of the road segment, in km
	private double length;
	
	static final double DEFAULT_LENGTH = 0.01;
	
	//Constructor
	public MapEdge(MapNode n1, MapNode n2, String roadName, String roadType, double length) {
		// TODO Auto-generated constructor stub
		start = n1;
		end = n2;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	//Return the MapNode for the end point
	public MapNode getEndNode() {
		return end;
	}
	//Return the length
	public double getLength() {
		return length;
	}
	//Return road name
	public String getRoadName() {
		return roadName;
	}
	//Given one node in an edge, return the other node
	public MapNode getOtherNode(MapNode node) {
		//if given an 'start', return 'end'
		if(node.equals(start)) {
			return end;
		}
		//if given an 'end', return 'start'			
		else if(node.equals(end)) {
			return start;
		}
		throw new IllegalArgumentException("Looking for " +"a point that is not in the edge"); 
	}
	
	
	
}

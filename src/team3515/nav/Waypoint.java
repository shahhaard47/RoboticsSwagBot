package team3515.nav;

public class Waypoint {
	public double x;
	public double y;
	public double z;
	
	public Waypoint nearest = null;
	
	/**
	 * waypoint
	 * 
	 * a position defined by x and y
	 * additionally can have a z set
	 */
	public Waypoint(double px, double py, double az){
		this.x = px;
		this.y = py;
		this.z = az;
	}
	
	public Waypoint(double px, double py){
		this.x = px;
		this.y = py;
		this.z = (double) 0.0;
	}
}
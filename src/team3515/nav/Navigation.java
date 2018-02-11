package team3515.nav;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Navigation {
	private DifferentialDrive drive;
	// current position
	public RobotPos position;
	// target position
	public Waypoint dest = null;
	
	// list of waypoints
	LinkedList<Waypoint> waypoints;
	// pending waypoint
	Waypoint pending;
	
	/**
	 * navigation
	 * 
	 * used for driving between points
	 */
	public Navigation(RobotPos p, DifferentialDrive drive){
		this.position = p;
		this.drive = drive;
		this.waypoints = new LinkedList<Waypoint>();
		this.waypoints.add(new Waypoint(0,0,0));
	}
	
	/**
	 * setDest
	 * 
	 * set a target destination
	 */
	void setDest(Waypoint d) {
		this.dest = d;
	}
	
	/**
	 * drive
	 * 
	 * sets motors to drive the robot to the destination
	 * 
	 */
	/*
	boolean drive() {
		boolean done = false;
		// if we have set a destination
		if(this.dest != null) {
			// are we there yet
			if(this.position.posX != dest.x || this.position.posY != dest.y) {
				// TODO drive towards dest
				
				// turn towards dest
				
				// drive to dest
			}
			else {
				done = true;
				// TODO add pending waypoint to list
			}
		}
		else {
			done = true;
		}
		
		return done;
	}
	*/
	
	public boolean driveTo(double x, double y, double z) {
		boolean done = false;
		double driveS = (double) 0.0;
		double driveT = (double) 0.0;
		
		double diffX = x - this.position.posX;
		double diffY = y - this.position.posY;
		//double diffZ = this.position.angZ - z;
		
		if(Math.abs(diffX) > (double) 0.0 || Math.abs(diffY) > (double) 0.0) {
			// find angle to dest
			double targetAngle = Math.atan(diffY/diffX);
			double angleMod = targetAngle - this.position.angZ;
			if(angleMod > 0) {
				driveT = (double) 0.4;
			}
			else {
				driveT = (double) -0.4;
			}
			driveS = (double) 0.5;
		}
		else {
			done = true;
		}
		
		this.drive.arcadeDrive(driveS, driveT);
		return done;
	}
}
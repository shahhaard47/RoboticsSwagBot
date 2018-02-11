package team3515.nav;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotPos {
	// imu
	ADIS16448_IMU imu;
	
	// position data
	public double posX;
	public double posY;
	public double angZ;
	
	// movement data
	private double vxi = 0.0;
	private double vyi = 0.0;
	private double axi = 0.0;
	private double ayi = 0.0;
	private double ti = 0.0;
	private double lastAngle = 0.0;
	
	public RobotPos(ADIS16448_IMU imu){
		this.reset();
		this.imu = imu;
	}
	
	/**
	 * reset
	 * 
	 * make the current position of the robot the origin/home position
	 */
	public void reset() {
		this.posX = 0.0;
		this.posY = 0.0;
		this.angZ = 0.0;
		
		this.vxi = 0.0;
		this.vyi = 0.0;
		this.axi = 0.0;
		this.ayi = 0.0;
		this.lastAngle = 0.0;
		this.ti = (double)System.currentTimeMillis();
	}
	
	/**
	 * move
	 * 
	 * increment the position by the given amounts
	 * 
	 * dx, dy, dz
	 */
	void move(double x, double y, double angle) {
		this.posX += x;
		this.posY += y;
		this.angZ += angle;
		this.angZ = capAngle(this.angZ);
	}
	
	/**
	 * capAngle
	 * 
	 * limit angle to between 0 and 360 degrees
	 */
	double capAngle(double angle) {
		while(angle < (double)0) {
			angle += 360.0;
		}
		// maybe change to >=
		while(angle > (double)360.0) {
			angle -= 360.0;
		}
		
		return angle;
	}
	
	/**
	 * resetGyro
	 * 
	 * keeps the last angle equal to the gyro when it is reset.
	 */
	void resetGyro() {
		this.lastAngle = 0.0;
	}
	
	/**
	 * dtor
	 * 
	 * convert an angle in degrees to radians.
	 * @param angle
	 * @return
	 */
	double dtor(double angle) {
		angle = capAngle(angle);
		angle = Math.PI * angle / ((double)180.0);
		return angle;
	}
	
	/**
	 * updatePos
	 * 
	 * find the difference in the robots position using imu data and time
	 */
	public void updatePos(Spark leftSpark, Spark rightSpark) {
		// get acceleration, angle, time
		double ax = imu.getAccelX() * (double)(-9.81);
		ax = (Math.abs(ax) < 0.25) ? 0 : ax;
		double ay = imu.getAccelY() * (double)(-9.81);
		ay = (Math.abs(ay) < 0.27) ? 0 : ay;
		double angle = imu.getAngleZ();
		double time = (double)System.currentTimeMillis();
		// get change in time and angle
		double dt = (time - ti)/1000.0; //change in time
		double dz = angle - lastAngle; //change in angle
		// find x : with respect to accelerometer
		double vxf = vxi + dt * (ax + axi) / 2;
		double dx = dt * (vxi + vxf) / 2;
		dx = (Math.abs(dx) < 0.01) ? 0 : dx;
		// find y : with respect to accelerometer
		double vyf = vyi + dt * (ay + ayi) / 2;
		
		double leftSpeed = leftSpark.get();
		double rightSpeed = rightSpark.get();
		double average = (leftSpeed + rightSpeed) / 2;
		
		// low-pass filter velocity
		vxf = (ax == 0 && average == (double)0.0) ? 0 : vxf;
		vyf = (ay == 0) ? 0 : vyf;
		
		double dy = dt * (vyi + vyf) / 2;
		dy = (Math.abs(dy) < 0.01) ? 0 : dy;
		// final angle
		double anglef = this.angZ + dz;
		SmartDashboard.putNumber("vxf", vxf);
		SmartDashboard.putNumber("vyf", vyf);
		SmartDashboard.putNumber("dx", dx);
		SmartDashboard.putNumber("dy", dy);
		
		// find chord lenght when turning
		double cx = dx; // defualt to straight line
		double cy = dy;
		double cz = anglef; // angle of motion
		if(dz != (double)0.0) {
			// find radius of turning circle
			// arc = 2*pi*r*(ang/360)
			// arc*360 = 2*pi*r*dz
			// arc*720*pi = r*dz
			double rx = (dx * ((double)720.0) * Math.PI) / dz;
			double ry = (dy * ((double)720.0) * Math.PI) / dz;
			
			// calculate chord
			double rad = dtor(dz);
			cx = 2 * rx * Math.sin(rad / 2);
			cy = 2 * ry * Math.sin(rad / 2);
			// find angle of motion relative to robotPos
			cz = (this.angZ + anglef) / 2;
		}
		
		// change movement so its relative to position data
		// TODO check
		double radf = dtor(cz);
		SmartDashboard.putNumber("cx", cx);
		SmartDashboard.putNumber("cy", cy);
		double x = Math.cos(radf) * cx - Math.sin(radf) * cy;
		double y = Math.sin(radf) * cx + Math.cos(radf) * cy;
		
		// update position data
		this.move(x, y, dz);
		
		// update make new data into last
		axi = ax;
		ayi = ay;
		vxi = vxf;
		vyi = vyf;
		
		lastAngle = angle;
		ti = time;
		
		// loop frequency
		double frq = (double)1.0 / dt;
		SmartDashboard.putNumber("Update Frequency", frq);
		SmartDashboard.putNumber("RobotX", this.posX);
		SmartDashboard.putNumber("RobotY", this.posY);
		SmartDashboard.putNumber("RobotZ", this.angZ);
		
	}
}
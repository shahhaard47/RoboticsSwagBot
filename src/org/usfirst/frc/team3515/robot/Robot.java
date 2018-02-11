/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//FIGURE OUT HOW TO MEASURE DISTANCE!!! 

package org.usfirst.frc.team3515.robot;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import team3515.nav.Navigation;
import team3515.nav.RobotPos;

import com.analog.adis16448.frc.ADIS16448_IMU;

import org.usfirst.frc.team3515.robot.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
@SuppressWarnings("deprecation")
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	//Camera Initializations
	private static final int IMG_WIDTH = 320;
    private static final int IMG_HEIGHT = 240;
    private VisionThread visionThread;
    private double centerX = 0.0;
    private DifferentialDrive drive;
    private final Object imgLock = new Object();

    final double FOCAL = 380.7692307692; 
	final int ORIG_W = 13; //inches
    
	private Joystick myStick;
	
	public static final ADIS16448_IMU imu = new ADIS16448_IMU();

	//3515
	public RobotPos position;
	public Navigation nav;

	Spark leftSpark;
	Spark rightSpark;
	
//	Talons to control rope rechanicasim 	
	Talon rope = new Talon(1);
	Talon rope2 = new Talon(2);
	
//	Talons for intake mechinism
	Talon intake = new Talon(3);
	Talon intake2 = new Talon(4);
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("CenterX", Integer.toString((int) centerX));
		SmartDashboard.putData("Auto choices", m_chooser);
//		SmartDashboard.putData("CenterX", centerX);
		
		//xBox controller aka joystick
		myStick = new Joystick(0);
		
		
		
		//USB Camera
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		
		visionThread = new VisionThread(camera, new FindRedAreas(), pipeline -> {
	         if (!pipeline.filterContoursOutput().isEmpty()) {
	             Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	             int pixel_width = r.width;
	             SmartDashboard.putString("Pixel Width", Integer.toString(pixel_width));
	             double distance = FOCAL*ORIG_W/pixel_width;
	             SmartDashboard.putString("Distance from box:", Double.toString(distance));
	             synchronized (imgLock) {
	                 centerX = r.x + (r.width / 2);
	             }
	         } });
		visionThread.start();
		
		leftSpark = new Spark(0);
		rightSpark = new Spark(9);
		
		drive = new DifferentialDrive(leftSpark, rightSpark);
		position = new RobotPos(imu);
		nav = new Navigation(position,drive);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		
		
	}
	
	

	@Override
	public void robotPeriodic() {
		// TODO Auto-generated method stub
		super.robotPeriodic();
		SmartDashboard.putNumber("Gyro-X", imu.getAngleX());
		SmartDashboard.putNumber("Gyro-Y", imu.getAngleY());
		SmartDashboard.putNumber("Gyro-Z", imu.getAngleZ());

		SmartDashboard.putNumber("Accel-X", imu.getAccelX());
		SmartDashboard.putNumber("Accel-Y", imu.getAccelY());
		SmartDashboard.putNumber("Accel-Z", imu.getAccelZ());

		SmartDashboard.putNumber("Pitch", imu.getPitch());
		SmartDashboard.putNumber("Roll", imu.getRoll());
		SmartDashboard.putNumber("Yaw", imu.getYaw());

		SmartDashboard.putNumber("Pressure: ", imu.getBarometricPressure());
		SmartDashboard.putNumber("Temperature: ", imu.getTemperature()); 

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
//		switch (m_autoSelected) {
//			case kCustomAuto:
//				// Put custom auto code here
//				break;
//			case kDefaultAuto:
//			default:
//				// Put default auto code here
//				break;
//		}
		
		double centerX;
	     synchronized (imgLock) {
	         centerX = this.centerX;
	     }
	     double turn = centerX - (IMG_WIDTH / 2);
	     drive.arcadeDrive(0.4, turn * -0.005);
	     
	}
	
	double leftSpeed, rightSpeed;
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		if (myStick.getRawButton(2)){
			driveStraight(imu.getAngleZ(), 0.6, 6);
		}
//		if (myStick.getRawButton(5)){
//			try {
//				driveDist(1);
//			} catch (Exception e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			} //cm
//		}
		
		if(myStick.getRawButton(1)) {
//			nav.driveTo(0, 0, 0);
		}
		else {
			drive.tankDrive(leftSpeed = (-1)*myStick.getRawAxis(1), rightSpeed = (-1)*myStick.getRawAxis(5));
		}
		position.updatePos(leftSpark, rightSpark);
		if(myStick.getRawButton(4)) {
			imu.reset();
			position.reset();
		}
		if (myStick.getRawButton(3)){
			imu.calibrate();
		}
		
		if (myStick.getRawButton(6))
		{
			intake.setSpeed(0.4);
			intake2.setSpeed(0.4);
		}
		else if (myStick.getRawButton(5))
		{
			intake.setSpeed(-0.4);
			intake2.setSpeed(-0.4);
		}
		else {
			intake.setSpeed(0);
			intake2.setSpeed(0);
		}
		// STILL NEED: we need to get the actual hight for the scale and switch and the exchange... .
		double speedElevator;
		if ((speedElevator = myStick.getRawAxis(2)) != 0){
			// go up
			rope.setSpeed(speedElevator);
			rope2.setSpeed(speedElevator);
		}else if((speedElevator = myStick.getRawAxis(3)) != 0){
			// go down
			speedElevator *= -1;
			rope.setSpeed(speedElevator);
			rope2.setSpeed(speedElevator);
		}else {
			rope.setSpeed(0);
			rope2.setSpeed(0);
		}
	}
	
	private void driveDist(int dist) throws InterruptedException, FileNotFoundException, UnsupportedEncodingException {
		double xi = 0;
		double vi = 0;
		double ai = 0;
		double ti = System.nanoTime();
		double xf, vf, af, tf;
		double dt;
		xf = 0;
		vf = 0;
		boolean keep_driving = false;
		SmartDashboard.putBoolean("keep_driving", keep_driving);
		Thread.sleep(100);
		
		PrintWriter writer = new PrintWriter("~/measure_distance.csv", "UTF-8");
		
		writer.println("");
		
		
		while (keep_driving = ((double)Math.abs(xf) < (double)1.0)) {
			SmartDashboard.putBoolean("keep_driving", keep_driving);
			SmartDashboard.putNumber("dist", dist);
			
			drive.arcadeDrive(0.6, 0);
			af = imu.getAccelX() * 9.81;
			tf = System.nanoTime();
			dt = tf - ti;
			vf += (af - ai) / dt;
			xf += (vf - vi) / dt;
			SmartDashboard.putNumber("xf", xf);
			
			if (xf >= dist) break;
			
			if (myStick.getRawButton(6)) break;
			
			//Initial values
			vi = vf; ai = af; ti = tf;
		}
		
		writer.close();
	}
	
	private void driveStraight(double init_angle, double speed, int button) {
		final double correction = 0.05;
		double turn;
		while (myStick.getRawButton(button)){
			turn = init_angle - imu.getAngleZ();
			drive.arcadeDrive(speed, -turn*correction);
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}

package com.team1323.frc2018.loops;

import java.util.ArrayList;
import java.util.List;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.RobotState;
import com.team1323.frc2018.vision.TargetInfo;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightProcessor implements Loop{
	static LimelightProcessor instance = new LimelightProcessor();
	edu.wpi.first.networktables.NetworkTable table;
	RobotState robotState = RobotState.getInstance();
	NetworkTableEntry ledMode;
	NetworkTableEntry pipeline;
	NetworkTableEntry camMode;
	NetworkTableEntry tx;
	NetworkTableEntry ty;
	NetworkTableEntry ta;
	NetworkTableEntry tv;
	
	public static LimelightProcessor getInstance(){
		return instance;
	}
	
	public LimelightProcessor(){
	}
	
	@Override 
	public void onStart(double timestamp){
		table = NetworkTableInstance.getDefault().getTable("limelight");
		ledMode = table.getEntry("ledMode");
		pipeline = table.getEntry("pipeline");
		camMode = table.getEntry("camMode");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
		tv = table.getEntry("tv");
		setPipeline(0);
	}
	
	@Override 
	public void onLoop(double timestamp){
		double targetOffsetAngle_Horizontal = tx.getDouble(0);
		double targetOffsetAngle_Vertical = ty.getDouble(0);
		double targetArea = ta.getDouble(0);
		boolean targetInSight = (tv.getDouble(0) == 1.0) ? true : false;
		List<TargetInfo> targets = new ArrayList<TargetInfo>(1);
		if(targetInSight){
			targets.add(new TargetInfo(Math.tan(Math.toRadians(targetOffsetAngle_Horizontal)), Math.tan(Math.toRadians(targetOffsetAngle_Vertical))));
		}
		//System.out.println(Math.tan(Math.toRadians(targetOffsetAngle_Horizontal)) + ", " + Math.tan(Math.toRadians(targetOffsetAngle_Vertical)));
		robotState.addVisionUpdate(timestamp, targets);
		robotState.setAngleToCube(targetOffsetAngle_Horizontal);
		SmartDashboard.putNumber("Limelight Angle", targetOffsetAngle_Horizontal);
		
		double distance = (Constants.kTargetHeight - Constants.kCameraZOffset) / Math.tan(Math.toRadians(targetOffsetAngle_Vertical));
		//System.out.println(distance);
	}
	
	@Override
	public void onStop(double timestamp){
		
	}
	
	public void blink(){
		if(ledMode.getDouble(0) != 2)
			ledMode.setNumber(2);
	}
	
	public void ledOn(boolean on){
		if(ledMode.getDouble(1) != 0 && on)
			ledMode.setNumber(0);
		else if(ledMode.getDouble(0) != 1 && !on)
			ledMode.setNumber(1);
	}
	
	public void setDriverMode(){
		camMode.setNumber(1);
	}
	
	public void setVisionMode(){
		camMode.setNumber(0);
	}
	
	public void setPipeline(int id){
		pipeline.setNumber(id);
	}
}

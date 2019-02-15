package com.team1323.frc2019.loops;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2019.RobotState;
import com.team1323.frc2019.vision.TargetInfo;

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
	public List<NetworkTableEntry> target1, target2, combinedTarget;
	NetworkTableEntry camtran;
	
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
		target1 = Arrays.asList(table.getEntry("tx0"), table.getEntry("ty0"),
			table.getEntry("ta0"));
		target2 = Arrays.asList(table.getEntry("tx1"), table.getEntry("ty1"),
			table.getEntry("ta1"));
		combinedTarget = Arrays.asList(table.getEntry("tx"), table.getEntry("ty"),
			table.getEntry("ta"), table.getEntry("tv"));
		camtran = table.getEntry("camtran");
		setPipeline(0);
	}
	
	@Override 
	public void onLoop(double timestamp){
		List<TargetInfo> targets = new ArrayList<TargetInfo>(3);
		if(seesTarget()){
			targets.add(getTargetInfo(target1));
			targets.add(getTargetInfo(target2));
			targets.add(new TargetInfo(Math.tan(Math.toRadians(combinedTarget.get(0).getDouble(0))), Math.tan(Math.toRadians(combinedTarget.get(1).getDouble(0)))));
		}

		SmartDashboard.putNumber("Limelight Pitch", camtran.getDoubleArray(new double[]{0.0,0.0,0.0,0.0,0.0,0.0})[3]);

		robotState.addVisionUpdate(timestamp, targets);		
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

	private boolean seesTarget(){
		boolean targetInSight = (combinedTarget.get(3).getDouble(0) == 1.0) ? true : false;
		return targetInSight;
	}

	public TargetInfo getTargetInfo(List<NetworkTableEntry> target){
		double nx = target.get(0).getDouble(0);
		double ny = target.get(1).getDouble(0);
		double vpw = 2.0 * Math.tan(Math.toRadians(27.0));
		double vph = 2.0 * Math.tan(Math.toRadians(24.85));
		double x = vpw / 2.0 * nx;
		double y = vph / 2.0 * ny;
		double ax = Math.atan2(x, 1.0);
		double ay = Math.atan2(y, 1.0);

		return new TargetInfo(Math.tan(ax), Math.tan(ay));
	}
}

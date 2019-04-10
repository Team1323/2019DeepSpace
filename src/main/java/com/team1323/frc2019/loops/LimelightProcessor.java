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
	NetworkTableEntry stream;
	public List<NetworkTableEntry> target1, target2, combinedTarget;
	public NetworkTableEntry cornerX, cornerY;

	boolean updatesAllowed = true;
	public void enableUpdates(boolean enable){
		updatesAllowed = enable;
	}
	
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
		stream = table.getEntry("stream");
		setStreamMode(2);//0
		target1 = Arrays.asList(table.getEntry("tx0"), table.getEntry("ty0"),
			table.getEntry("ta0"));
		target2 = Arrays.asList(table.getEntry("tx1"), table.getEntry("ty1"),
			table.getEntry("ta1"));
		combinedTarget = Arrays.asList(table.getEntry("tx"), table.getEntry("ty"),
			table.getEntry("ta"), table.getEntry("tv"));
		cornerX = table.getEntry("tcornx");
		cornerY = table.getEntry("tcorny");
		setPipeline(Pipeline.CLOSEST);
	}
	
	@Override 
	public void onLoop(double timestamp){
		List<TargetInfo> targets = new ArrayList<TargetInfo>();
		if(seesTarget() && updatesAllowed){
			//List<TargetInfo> tapeStrips = getTargetInfos();
			//targets.add(tapeStrips.get(0));
			//targets.add(tapeStrips.get(1));

			targets.add(getTargetInfo(target1));
			targets.add(getTargetInfo(target2));
			targets.add(new TargetInfo(Math.tan(Math.toRadians(combinedTarget.get(0).getDouble(0))), Math.tan(Math.toRadians(combinedTarget.get(1).getDouble(0)))));
		}

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

	public void setStreamMode(int id){
		stream.setNumber(id);
	}
	
	public void setPipeline(int id){
		pipeline.setNumber(id);
	}

	public void setPipeline(Pipeline p){
		setPipeline(p.id);
		System.out.println("Pipeline set to " + p.id);
	}

	public enum Pipeline{
		LEFTMOST(0), RIGHTMOST(1), CLOSEST(2),
		LOWEST(3), HIGHEST(4);

		int id;
		private Pipeline(int id){
			this.id = id;
		}
	}

	private boolean seesTarget(){
		boolean targetInSight = (combinedTarget.get(3).getDouble(0) == 1.0) ? true : false;
		return targetInSight;
	}

	public List<TargetInfo> getTargetInfos(){
		List<Double> leftXs = new ArrayList<>();
		for(int i=0; i<4; i++){
			leftXs.add(cornerX.getDoubleArray(new double[8])[i]);
		}
		double firstAverageX = 0.0;
		for(double x : leftXs){
			firstAverageX += x;
		}
		firstAverageX /= 4.0;

		List<Double> rightXs = new ArrayList<>();
		for(int i=4; i<8; i++){
			rightXs.add(cornerX.getDoubleArray(new double[8])[i]);
		}
		double secondAverageX = 0.0;
		for(double x : rightXs){
			secondAverageX += x;
		}
		secondAverageX /= 4.0;

		List<Double> leftYs = new ArrayList<>();
		for(int i=0; i<4; i++){
			leftYs.add(cornerY.getDoubleArray(new double[8])[i]);
		}
		double firstAverageY = 0.0;
		for(double y : leftYs){
			firstAverageY += y;
		}
		firstAverageY /= 4.0;

		List<Double> rightYs = new ArrayList<>();
		for(int i=4; i<8; i++){
			rightYs.add(cornerY.getDoubleArray(new double[8])[i]);
		}
		double secondAverageY = 0.0;
		for(double y : rightYs){
			secondAverageY += y;
		}
		secondAverageY /= 4.0;

		List<TargetInfo> targets = new ArrayList<>();
		targets.add(getTargetInfo((1.0/160.0)*(firstAverageX - 159.5), (1.0/120.0)*(119.5 - firstAverageY)));
		targets.add(getTargetInfo((1.0/160.0)*(secondAverageX - 159.5), (1.0/120.0)*(119.5 - secondAverageY)));

		return targets;
	}

	public TargetInfo getTargetInfo(double nx, double ny){
		double vpw = 2.0 * Math.tan(Math.toRadians(29.8));// 27.0 29.8
		double vph = 2.0 * Math.tan(Math.toRadians(24.85));//24.85  20.5
		double x = vpw / 2.0 * nx;
		double y = vph / 2.0 * ny;
		double ax = Math.atan2(x, 1.0);
		double ay = Math.atan2(y, 1.0);

		return new TargetInfo(Math.tan(ax), Math.tan(ay));
	}

	public TargetInfo getTargetInfo(List<NetworkTableEntry> target){
		double nx = target.get(0).getDouble(0);
		double ny = target.get(1).getDouble(0);
		double vpw = 2.0 * Math.tan(Math.toRadians(29.8));// 27.0 29.8
		double vph = 2.0 * Math.tan(Math.toRadians(24.85));//24.85  20.5
		double x = vpw / 2.0 * nx;
		double y = vph / 2.0 * ny;
		double ax = Math.atan2(x, 1.0);
		double ay = Math.atan2(y, 1.0);

		return new TargetInfo(Math.tan(ax), Math.tan(ay));
	}
}

package com.team1323.frc2019.auto.actions;

import com.team1323.frc2019.subsystems.Swerve;

public class WaitForHeadingAction implements Action{
	Swerve swerve;
	double lowThreshold;
	double highThreshold;
	
	public WaitForHeadingAction(double lowThreshold, double highThreshold){
		swerve = Swerve.getInstance();
		this.lowThreshold = lowThreshold;
		this.highThreshold = highThreshold;
	}
	
	@Override
	public boolean isFinished(){
		double heading = swerve.getPose().getRotation().getUnboundedDegrees();
		return heading >= lowThreshold && heading <= highThreshold;
	}
	
	@Override
	public void start(){
	}
	
	@Override
	public void update(){
	}
	
	@Override
	public void done(){
	}
}

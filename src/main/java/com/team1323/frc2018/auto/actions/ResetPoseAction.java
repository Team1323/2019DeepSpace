package com.team1323.frc2018.auto.actions;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.subsystems.Swerve;
import com.team254.lib.geometry.Pose2d;

public class ResetPoseAction extends RunOnceAction{
	private Pose2d newPose;
	
	public ResetPoseAction(Pose2d newPose){
		this.newPose = newPose;
	}

	public ResetPoseAction(boolean left){
		newPose = left ? Constants.kRobotLeftStartingPose : Constants.kRobotRightStartingPose;
	}

	@Override
	public void runOnce() {
		Swerve.getInstance().zeroSensors(newPose);
	}

}

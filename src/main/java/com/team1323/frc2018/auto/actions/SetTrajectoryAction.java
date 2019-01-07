package com.team1323.frc2018.auto.actions;

import com.team1323.frc2018.subsystems.Swerve;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;

public class SetTrajectoryAction extends RunOnceAction{
	Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
    double goalHeading;
    double rotationScalar;
	Swerve swerve;
	
	public SetTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goalHeading, double rotationScalar){
		this.trajectory = trajectory;
        this.goalHeading = goalHeading;
        this.rotationScalar = rotationScalar;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public synchronized void runOnce(){
		swerve.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)), goalHeading, rotationScalar);
	}
}

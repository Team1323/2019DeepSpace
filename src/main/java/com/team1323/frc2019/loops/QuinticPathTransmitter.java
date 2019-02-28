package com.team1323.frc2019.loops;

import java.util.ArrayList;
import java.util.List;

import com.team1323.frc2019.Constants;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class QuinticPathTransmitter implements Loop{
	private static QuinticPathTransmitter instance = new QuinticPathTransmitter();
	public static QuinticPathTransmitter getInstance(){
		return instance;
	}
	
	public QuinticPathTransmitter(){
	}
	
	private List<TrajectoryIterator<TimedState<Pose2dWithCurvature>>> remainingTrajectories = new ArrayList<>();
	private TrajectoryIterator<TimedState<Pose2dWithCurvature>> currentTrajectory;
	private double t = 0;
	private boolean defaultCookReported = false;

	private double minAccel = 0.0;
	
	private double startingTime = 0.0;
	
	public void addPath(Trajectory<TimedState<Pose2dWithCurvature>> path){
		remainingTrajectories.add(new TrajectoryIterator<>(new TimedView<>(path)));
		currentTrajectory = null;
	}

	public void addPaths(List<Trajectory<TimedState<Pose2dWithCurvature>>> paths){
		paths.forEach((p) -> addPath(p));
	}

	public double getTotalPathTime(List<Trajectory<TimedState<Pose2dWithCurvature>>> paths){
        double total = 0.0;

        for(Trajectory<TimedState<Pose2dWithCurvature>> path : paths){
            total += path.getLastState().t();
        }

        return total;
    }

	@Override
	public void onStart(double timestamp) {
		
	}

	@Override
	public void onLoop(double timestamp) {
		if(currentTrajectory == null){
			if(remainingTrajectories.isEmpty()){
				return;
			}
			
			currentTrajectory = remainingTrajectories.remove(0);
			defaultCookReported = false;
			t = 0;
			startingTime = timestamp;
		}
		
		t = timestamp - startingTime;
		TimedState<Pose2dWithCurvature> state = currentTrajectory.preview(t).state();
		Translation2d pos = state.state().getTranslation();
	SmartDashboard.putNumberArray("Path Pose", new double[]{pos.x(), pos.y(), 0.0, /*Math.abs(state.acceleration()) / 10.0*/Math.abs(state.velocity()) / Constants.kSwerveMaxSpeedInchesPerSecond}); 
		
		//System.out.println("Accel: " + state.acceleration());
		if(state.acceleration() < minAccel)
			minAccel = state.acceleration();

		if(state.acceleration() < 0.0 && !defaultCookReported){
			//System.out.println("Optimal default cook: " + state.velocity());
			defaultCookReported = true;
		}

	    if(t >= currentTrajectory.trajectory().getLastState().t()){
			//System.out.println("Path should take " + currentTrajectory.trajectory().getLastState().t() + " seconds");
			//System.out.println("Min accel: " + minAccel);
	    	currentTrajectory = null;
	    }
	}

	@Override
	public void onStop(double timestamp) {
		
	}

}

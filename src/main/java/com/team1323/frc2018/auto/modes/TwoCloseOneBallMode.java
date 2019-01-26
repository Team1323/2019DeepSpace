package com.team1323.frc2018.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.auto.AutoModeBase;
import com.team1323.frc2018.auto.AutoModeEndedException;
import com.team1323.frc2018.auto.actions.ResetPoseAction;
import com.team1323.frc2018.auto.actions.SetTrajectoryAction;
import com.team1323.frc2018.auto.actions.WaitAction;
import com.team1323.frc2018.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class TwoCloseOneBallMode extends AutoModeBase {
	Superstructure s;
    Intake intake;
    
    final boolean left;
    final double directionFactor;

    private List<Trajectory<TimedState<Pose2dWithCurvature>>> paths = Arrays.asList(trajectories.startToCloseHatch.get(true),
        trajectories.closeHatchToHumanLoader.get(true), trajectories.humanLoaderToCloseHatch.get(true), trajectories.closeHatchToBall.get(true),
            trajectories.ballToRocketPort.get(true));

    @Override 
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths(){
        return paths;
    }

	public TwoCloseOneBallMode(boolean left) {
        s = Superstructure.getInstance();
        intake = Intake.getInstance();
        this.left = left;
        directionFactor = left ? -1.0 : 1.0;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();

        runAction(new ResetPoseAction(Constants.kRobotStartingPose));
        runAction(new SetTrajectoryAction(trajectories.startToCloseHatch.get(left), 30.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.closeHatchToHumanLoader.get(left), 180.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.humanLoaderToCloseHatch.get(left), 30.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.closeHatchToBall.get(left), 45.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.ballToRocketPort.get(left), 90.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.rocketPortToHumanLoader.get(left), 180.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
	}
	
}
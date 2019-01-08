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

    private static List<Trajectory<TimedState<Pose2dWithCurvature>>> paths = Arrays.asList(trajectories.startToCloseHatch,
        trajectories.closeHatchToHumanLoader, trajectories.humanLoaderToCloseHatch, trajectories.closeHatchToBall,
            trajectories.ballToRocketPort);

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return paths;
    }

	public TwoCloseOneBallMode() {
        s = Superstructure.getInstance();
        intake = Intake.getInstance();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();

        runAction(new ResetPoseAction(Constants.kRobotStartingPose));
        runAction(new SetTrajectoryAction(trajectories.startToCloseHatch, -30.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.closeHatchToHumanLoader, -180.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.humanLoaderToCloseHatch, -30.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.closeHatchToBall, -45.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.ballToRocketPort, -90.0, 1.0));

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
	}
	
}
package com.team1323.frc2019.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2019.auto.AutoModeBase;
import com.team1323.frc2019.auto.AutoModeEndedException;
import com.team1323.frc2019.auto.actions.ResetPoseAction;
import com.team1323.frc2019.auto.actions.SetTrajectoryAction;
import com.team1323.frc2019.auto.actions.WaitAction;
import com.team1323.frc2019.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2019.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class FarCloseBallMode extends AutoModeBase {
	Superstructure s;
    
    final boolean left;
    final double directionFactor;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.startToFarHatch.get(left), trajectories.farHatchToHumanLoader.get(left), 
            trajectories.humanLoaderToCloseHatch.get(left), trajectories.closeHatchToBall.get(left),
            trajectories.ballToRocketPort.get(left));
    }

	public FarCloseBallMode(boolean left) {
        s = Superstructure.getInstance();
        this.left = left;
        directionFactor = left ? -1.0 : 1.0;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();

        runAction(new ResetPoseAction(left));
        runAction(new SetTrajectoryAction(trajectories.startToFarHatch.get(left), 150.0 *directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.farHatchToHumanLoader.get(left), 180.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.humanLoaderToCloseHatch.get(left), 150.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.closeHatchToBall.get(left), 45.0 *directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.ballToRocketPort.get(left), 90.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
	}
	
}
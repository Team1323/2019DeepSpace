package com.team1323.frc2019.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2019.Constants;
import com.team1323.frc2019.RobotState;
import com.team1323.frc2019.auto.AutoModeBase;
import com.team1323.frc2019.auto.AutoModeEndedException;
import com.team1323.frc2019.auto.actions.RemainingProgressAction;
import com.team1323.frc2019.auto.actions.ResetPoseAction;
import com.team1323.frc2019.auto.actions.SetTrajectoryAction;
import com.team1323.frc2019.auto.actions.WaitAction;
import com.team1323.frc2019.auto.actions.WaitForDiskAction;
import com.team1323.frc2019.auto.actions.WaitForDistanceAction;
import com.team1323.frc2019.auto.actions.WaitForSuperstructureAction;
import com.team1323.frc2019.auto.actions.WaitForHeadingAction;
import com.team1323.frc2019.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2019.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2019.auto.actions.WaitToPassYCoordinateAction;
import com.team1323.frc2019.loops.LimelightProcessor;
import com.team1323.frc2019.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2019.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class CloseFarBallMode extends AutoModeBase {
	Superstructure s;
    
    final boolean left;
    final double directionFactor;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.startToCloseHatch.get(left), trajectories.closeHatchToHumanLoader.get(left), 
            trajectories.humanLoaderToFarHatch.get(left), trajectories.farHatchToBall.get(left),
            trajectories.ballToRocketPort.get(left));
    }

	public CloseFarBallMode(boolean left) {
        s = Superstructure.getInstance();
        this.left = left;
        directionFactor = left ? -1.0 : 1.0;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();

        runAction(new ResetPoseAction(left));
        runAction(new SetTrajectoryAction(trajectories.startToCloseHatch.get(left), 30.0 * directionFactor, 1.0));
        LimelightProcessor.getInstance().setPipeline(Pipeline.LEFTMOST);
        runAction(new WaitToPassYCoordinateAction(-46.25 - Constants.kRobotWidth));
        s.diskScoringState(Constants.kElevatorMidHatchHeight);
        runAction(new WaitForDistanceAction(Constants.closeHatchPosition.getTranslation(), 102.0));
        s.diskTrackingState(Constants.kElevatorMidHatchHeight, Rotation2d.fromDegrees(30.0 * directionFactor));
        runAction(new WaitForSuperstructureAction());
        runAction(new WaitAction(0.25));


        runAction(new SetTrajectoryAction(trajectories.closeHatchToHumanLoader.get(left), 180.0 * directionFactor, 0.75));
        runAction(new WaitAction(0.5));
        s.diskReceivingState();
        LimelightProcessor.getInstance().setPipeline(Pipeline.RIGHTMOST);
        runAction(new WaitToPassXCoordinateAction(96.0));
        runAction(new WaitForHeadingAction(-190.0, -160.0));
        s.humanLoaderTrackingState();
        runAction(new WaitForDiskAction(3.0));


        runAction(new SetTrajectoryAction(trajectories.humanLoaderToFarHatch.get(left), 150.0 * directionFactor, 1.0));
        LimelightProcessor.getInstance().setPipeline(Pipeline.RIGHTMOST);
        runAction(new RemainingProgressAction(3.75));
        s.diskScoringState(Constants.kElevatorMidHatchHeight);
        runAction(new RemainingProgressAction(1.25));
        s.diskTrackingState(Constants.kElevatorMidHatchHeight, Rotation2d.fromDegrees(150.0 * directionFactor));
        runAction(new WaitForSuperstructureAction());
        //runAction(new WaitAction(0.25));


        runAction(new SetTrajectoryAction(trajectories.farHatchToBall.get(left), 150.0 * directionFactor, 1.0));
        runAction(new WaitAction(0.5));
        s.diskScoringState(Constants.kElevatorLowHatchHeight);
        runAction(new WaitToFinishPathAction());
        /*runAction(new WaitAction(0.5));
        runAction(new SetTrajectoryAction(trajectories.ballToRocketPort.get(left), 90.0 * directionFactor, 1.0));
        runAction(new WaitToFinishPathAction());*/

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
	}
	
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2019.Constants;
import com.team1323.frc2019.RobotState;
import com.team1323.frc2019.auto.AutoModeBase;
import com.team1323.frc2019.auto.AutoModeEndedException;
import com.team1323.frc2019.auto.actions.ResetPoseAction;
import com.team1323.frc2019.auto.actions.SetTrajectoryAction;
import com.team1323.frc2019.auto.actions.WaitAction;
import com.team1323.frc2019.auto.actions.WaitForElevatorAction;
import com.team1323.frc2019.auto.actions.WaitForHeadingAction;
import com.team1323.frc2019.auto.actions.WaitForSuperstructureAction;
import com.team1323.frc2019.auto.actions.WaitForVisionAction;
import com.team1323.frc2019.auto.actions.WaitForVisionPositionAction;
import com.team1323.frc2019.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2019.loops.LimelightProcessor;
import com.team1323.frc2019.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2019.subsystems.DiskScorer;
import com.team1323.frc2019.subsystems.Superstructure;
import com.team1323.frc2019.subsystems.Swerve;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class MidCloseShipMode extends AutoModeBase{
    Superstructure s;

    final boolean left;
    final double directionFactor;

    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.startToMidShip.get(left), trajectories.midShipToHumanLoader.get(left),
            trajectories.humanLoaderToCloseShip.get(left), trajectories.closeShipToHumanLoader.get(left));
    }

	public MidCloseShipMode(boolean left) {
        s = Superstructure.getInstance();
        this.left = left;
        directionFactor = left ? -1.0 : 1.0;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();

        runAction(new ResetPoseAction(left ? new Pose2d(Constants.kRobotLeftStartingPose.getTranslation(), Rotation2d.fromDegrees(90.0)) : new Pose2d(Constants.kRobotRightStartingPose.getTranslation(), Rotation2d.fromDegrees(-90.0))));
        DiskScorer.getInstance().conformToState(DiskScorer.State.GROUND_DETECTED);
        runAction(new SetTrajectoryAction(trajectories.startToMidShip.get(left), -90.0 * directionFactor, 1.0));
        LimelightProcessor.getInstance().setPipeline(left ? Pipeline.LEFTMOST : Pipeline.RIGHTMOST);
        runAction(new WaitToPassXCoordinateAction((96.0 + Constants.kRobotWidth)));
        RobotState.getInstance().setXTarget(Constants.midShipPosition.getTranslation().x(), 7.0);
        s.diskScoringState(20.0, true);
        runAction(new WaitToPassXCoordinateAction(264.0));//282.55 267
        runAction(new WaitForElevatorAction(19.6, true));
        runAction(new WaitForVisionAction(2.0));
        runAction(new WaitForVisionPositionAction(Constants.midShipPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 4.0, 6.0))).getTranslation().x()));
        //LimelightProcessor.getInstance().setPipeline(Pipeline.CLOSEST);
        s.diskTrackingState(Constants.kElevatorLowHatchHeight, Rotation2d.fromDegrees(-90.0 * directionFactor), 42.0, new Translation2d(-6.0, Constants.kCurvedVisionYOffset), 36.0);
        runAction(new WaitForSuperstructureAction());
        Swerve.getInstance().setXCoordinate(Constants.midShipPosition.getTranslation().x());
        RobotState.getInstance().enableXTarget(false);
        runAction(new WaitAction(0.25));


        runAction(new SetTrajectoryAction(trajectories.midShipToHumanLoader.get(left), -180.0 * directionFactor, 0.75));
        runAction(new WaitAction(0.5));
        s.diskScoringState(12.4, false);
        LimelightProcessor.getInstance().setPipeline(Pipeline.RIGHTMOST);
        runAction(new WaitToPassXCoordinateAction(96.0));
        if(left)
            runAction(new WaitForHeadingAction(160.0, 190.0));
        else
            runAction(new WaitForHeadingAction(-190.0, -160.0));
        runAction(new WaitForVisionAction(3.0));
        s.humanLoaderTrackingState();
        runAction(new WaitForSuperstructureAction());
        Swerve.getInstance().setYCoordinate(directionFactor * -1.0 * Constants.humanLoaderPosition.getTranslation().y());
        Swerve.getInstance().setXCoordinate(Constants.kRobotHalfLength);


        runAction(new SetTrajectoryAction(trajectories.humanLoaderToCloseShip.get(left), -90.0 * directionFactor, 0.75));
        LimelightProcessor.getInstance().setPipeline(left ? Pipeline.RIGHTMOST : Pipeline.LEFTMOST);
        runAction(new WaitToPassXCoordinateAction((96.0 + Constants.kRobotWidth)));
        s.diskScoringState(20.0, true);
        runAction(new WaitToPassXCoordinateAction(238.0));//260.8
        runAction(new WaitForElevatorAction(19.6, true));
        runAction(new WaitForVisionAction(2.0));
        s.diskTrackingState(Constants.kElevatorLowHatchHeight, Rotation2d.fromDegrees(-90.0 * directionFactor), 42.0, new Translation2d(-8.0, Constants.kCurvedVisionYOffset), 30.0);
        runAction(new WaitForSuperstructureAction());
        Swerve.getInstance().setXCoordinate(Constants.closeShipPosition.getTranslation().x());
        runAction(new WaitAction(0.25));

        //runAction(new SetTrajectoryAction(trajectories.closeShipToHumanLoader.get(left), -180.0 * directionFactor, 1.0));
        Swerve.getInstance().setRobotCentricTrajectory(new Translation2d(-36.0, 0.0), -90.0 * directionFactor);

        System.out.println("Auto mode finished in " + currentTime() + " seconds");
	}
	
}

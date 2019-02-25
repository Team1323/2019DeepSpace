package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2019.Constants;
import com.team1323.frc2019.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 120.0;
    private static final double kMaxDecel = 72.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if(mTrajectorySet == null) {
        	double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
    static final Pose2d autoStartingPose = new Pose2d(Constants.kRobotLeftStartingPose.getTranslation().translateBy(new Translation2d(/*-0.5*/0.0, 0.0)), Rotation2d.fromDegrees(0.0));

    static final Pose2d closeHatchScoringPose = Constants.closeHatchPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 3.5, 0.0)));
    static final Pose2d farHatchScoringPose = Constants.farHatchPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 12.0, 0.0)));
    static final Pose2d humanLoaderPose = Constants.humanLoaderPosition.transformBy(Pose2d.fromTranslation(new Translation2d(Constants.kRobotHalfLength - 4.0, 2.0)));
    static final Pose2d ballIntakePose = new Pose2d(Constants.autoBallPosition.transformBy(Pose2d.fromTranslation(new Translation2d(Constants.kRobotHalfLength + 12.0, 0.0))).getTranslation(), Rotation2d.fromDegrees(0.0));
    static final Pose2d portScoringPose = Constants.rocketPortPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 6.0, 0.0)));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> straightPath;

        //Preliminary Auto Paths
        public final MirroredTrajectory startToCloseHatch;
        public final MirroredTrajectory closeHatchToHumanLoader;
        public final MirroredTrajectory humanLoaderToCloseHatch;
        public final MirroredTrajectory closeHatchToBall;
        public final MirroredTrajectory ballToRocketPort;
        public final MirroredTrajectory rocketPortToHumanLoader;

        public final MirroredTrajectory startToFarHatch;
        public final MirroredTrajectory farHatchToHumanLoader;
        public final MirroredTrajectory humanLoaderToFarHatch;
        public final MirroredTrajectory farHatchToBall;

        private TrajectorySet() {
            //Test Paths
            straightPath = getStraightPath();

            //Preliminary Auto Paths
            startToCloseHatch = new MirroredTrajectory(getStartToCloseHatch());
            closeHatchToHumanLoader = new MirroredTrajectory(getCloseHatchToHumanLoader());
            humanLoaderToCloseHatch = new MirroredTrajectory(getHumanLoaderToCloseHatch());
            closeHatchToBall = new MirroredTrajectory(getCloseHatchToBall());
            ballToRocketPort = new MirroredTrajectory(getBallToRocketPort());
            rocketPortToHumanLoader = new MirroredTrajectory(getRocketPortToHumanLoader());

            startToFarHatch = new MirroredTrajectory(getStartToFarHatch());
            farHatchToHumanLoader = new MirroredTrajectory(getFarHatchToHumanLoader());
            humanLoaderToFarHatch = new MirroredTrajectory(getHumanLoaderToFarHatch());
            farHatchToBall = new MirroredTrajectory(getFarHatchToBall());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStraightPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.kRobotLeftStartingPose);
            waypoints.add(Constants.kRobotLeftStartingPose.transformBy(Pose2d.fromTranslation(new Translation2d(72.0, 0.0))));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToCloseHatch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(autoStartingPose);
            //waypoints.add(autoStartingPose.transformBy(Pose2d.fromTranslation(new Translation2d(50.0, 0.0))));
            //waypoints.add(Constants.kRobotLeftRampExitPose);
            waypoints.add(closeHatchScoringPose);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, 24.0, kMaxVoltage, 36.0, 2);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCloseHatchToHumanLoader(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(closeHatchScoringPose);
            waypoints.add(humanLoaderPose);

            return generateTrajectory(true, waypoints, Arrays.asList(), 72.0, kMaxAccel, 24.0, kMaxVoltage, /*72.0*/60.0, 2);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getHumanLoaderToCloseHatch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(humanLoaderPose);
            waypoints.add(closeHatchScoringPose);

            return generateTrajectory(false, waypoints, Arrays.asList(), 96.0, kMaxAccel, 24.0, kMaxVoltage, 72.0, 2);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCloseHatchToBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(closeHatchScoringPose);
            waypoints.add(ballIntakePose);

            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, 48.0, kMaxVoltage, 72.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBallToRocketPort(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(ballIntakePose);
            waypoints.add(ballIntakePose.transformBy(Pose2d.fromTranslation(new Translation2d(96.0, 0.0))));
            waypoints.add(portScoringPose);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, 24.0, kMaxVoltage, 72.0, 2);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketPortToHumanLoader(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(portScoringPose);
            waypoints.add(humanLoaderPose);

            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, 24.0, kMaxVoltage, 72.0, 2);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToFarHatch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.kRobotLeftStartingPose);
            waypoints.add(farHatchScoringPose);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, 24.0, kMaxVoltage, 72.0, 2);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarHatchToHumanLoader(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(farHatchScoringPose);
            waypoints.add(new Pose2d(new Translation2d(210.0, -82.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(humanLoaderPose);

            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, 24.0, kMaxVoltage, 72.0, 2);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getHumanLoaderToFarHatch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(humanLoaderPose);
            waypoints.add(new Pose2d(new Translation2d(210.0, -82.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(farHatchScoringPose);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, 24.0, kMaxVoltage, 72.0, 2);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarHatchToBall(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(farHatchScoringPose);
            waypoints.add(ballIntakePose.transformBy(Pose2d.fromTranslation(new Translation2d(96.0, 0.0))));
            waypoints.add(ballIntakePose);

            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, 24.0, kMaxVoltage, 72.0, 2);
        }
    }
    
}

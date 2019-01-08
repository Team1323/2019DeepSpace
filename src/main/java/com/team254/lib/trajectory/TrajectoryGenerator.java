package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.CurvatureVelocityConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 12.5;
    private static final double kMaxAccel = 12.5;
    private static final double kMaxCentripetalAccelElevatorDown = 110.0 / 12.0;
    private static final double kMaxCentripetalAccel = 100.0 / 12.0;
    private static final double kMaxVoltage = 9.0;
    private static final double kFirstPathMaxVoltage = 9.0;
    private static final double kFirstPathMaxAccel = 10.0;
    private static final double kFirstPathMaxVel = 10.0;

    private static final double kSimpleSwitchMaxAccel = 100.0 / 12.0;
    private static final double kSimpleSwitchMaxCentripetalAccel = 80.0 / 12.0;
    private static final double kSimpleSwitchMaxVelocity = 120.0 / 12.0;

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
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    public static final Pose2d kSideStartPose = Pose2d.fromTranslation(new Translation2d(Constants.kRobotHalfWidth, 5.5 - Constants.kRobotHalfLength));

    public static final Pose2d kLeftScaleScorePose = new Pose2d(new Translation2d(22.75, Constants.kLeftSwitchCloseCorner.y() - Constants.kRobotHalfLength - 1.0),
        Rotation2d.fromDegrees(0.0));

    public static final Pose2d kSecondRightCubePose = new Pose2d(new Translation2d(Constants.kRightSwitchFarCorner.x() + 3.25, Constants.kRightSwitchFarCorner.y() + Constants.kRobotHalfLength - 3.25),
        Rotation2d.fromDegrees(90.0));

    //4 Cube Switch Auto?
    public static final Translation2d kLeftSwitchScoringPose = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - Constants.kRobotHalfLength - 4.0, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 1.0);
    public static final Translation2d kOuterCubeIntakingPose = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (3*Constants.kCubeWidth) - 2.0, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 5.1);
    public static final Translation2d kMiddleCubeIntakingPose = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.5, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 5.2);
    public static final Translation2d kLeftSwitchScoringPose2 = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.5, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 1.0);
    public static final Translation2d kBottomLeftIntakingPose = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.25, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 4.25);

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        //4 Cube Left Switch
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeFrontLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeFrontLeftToOuterCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeOuterToLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeFrontLeftToMiddleCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeMiddleToLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeFrontLeftToBottomLeft;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubebottomLeftToSwitch;

        //Poof assist switch + scale
        public final Trajectory<TimedState<Pose2dWithCurvature>> middleCubeToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> middleCubeToRightScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateMiddleCubeToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateMiddleCubeToRightScale;

        //Poof assist scale auto
        public final Trajectory<TimedState<Pose2dWithCurvature>> backOffLeftScale;

        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> straightPath;

        private TrajectorySet() {

            //4 Cube Switch Mode?
            fourCubeFrontLeftSwitch = getFourCubeFrontLeftSwitch();
            fourCubeFrontLeftToOuterCube = getFourCubeFrontLeftToOuterCube();
            fourCubeOuterToLeftSwitch = getFourCubeOuterToLeftSwitch();
            fourCubeFrontLeftToMiddleCube = getFourCubeFrontLeftToMiddleCube();
            fourCubeMiddleToLeftSwitch = getFourCubeMiddleToLeftSwitch();
            fourCubeFrontLeftToBottomLeft = getFourCubeFrontLeftToBottomLeft();
            fourCubebottomLeftToSwitch = getFourCubeBottomLeftToSwitch();

            //Switch + Scale Assist Modes
            middleCubeToLeftScale = getMiddleCubeToLeftScale();
            middleCubeToRightScale = getMiddleCubeToRightScale();
            alternateMiddleCubeToLeftScale = getAlternateMiddleCubeToLeftScale();
            alternateMiddleCubeToRightScale = getAlternateMiddleCubeToRightScale();

            //Poof Assist Mode
            backOffLeftScale = getBackOffLeftScale();

            //Test Paths
            straightPath = getStraightPath();
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMiddleCubeToRightScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kRightSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 2.25, Constants.kRightSwitchCloseCorner.y() - Constants.kRobotHalfWidth - 4.6), Rotation2d.fromDegrees(90.0)));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightSwitchCloseCorner.translateBy(new Translation2d(Constants.kRobotHalfWidth, Constants.kRobotHalfLength + 1.5))));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightScaleCorner.translateBy(new Translation2d(-2.75, 5.25))));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightScaleCorner.translateBy(new Translation2d(0.35, 5.25))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 8.0, 10.0, 2.0, kMaxVoltage, 7.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAlternateMiddleCubeToRightScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.65, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 5.2), Rotation2d.fromDegrees(90.0)));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightSwitchCloseCorner.translateBy(new Translation2d(Constants.kRobotHalfWidth - 1.0, Constants.kRobotHalfLength + 1.75))));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightScaleCorner.translateBy(new Translation2d(-2.75, 6.0))));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightScaleCorner.translateBy(new Translation2d(0.5, 6.0))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 8.0, 10.0, 2.0, kMaxVoltage, 7.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMiddleCubeToLeftScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.65, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 5.2), Rotation2d.fromDegrees(-90.0)));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftSwitchCloseCorner.translateBy(new Translation2d(Constants.kRobotHalfWidth, -Constants.kRobotHalfLength - 1.5))));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftScaleCorner.translateBy(new Translation2d(-2.75, -5.1))));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftScaleCorner.translateBy(new Translation2d(0.83, -5.1))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 8.0, 10.0, 2.0, kMaxVoltage, 7.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAlternateMiddleCubeToLeftScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kRightSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 2.25, Constants.kRightSwitchCloseCorner.y() - Constants.kRobotHalfWidth - 4.6), Rotation2d.fromDegrees(-90.0)));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftSwitchCloseCorner.translateBy(new Translation2d(Constants.kRobotHalfWidth, -Constants.kRobotHalfLength - 1.5))));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftScaleCorner.translateBy(new Translation2d(-2.75, -5.1))));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftScaleCorner.translateBy(new Translation2d(0.75, -5.1))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 8.0, 10.0, 2.0, kMaxVoltage, 7.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBackOffLeftScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftScaleScorePose);
            waypoints.add(kLeftScaleScorePose.transformBy(Pose2d.fromTranslation(new Translation2d(-7.0, -0.5))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 6.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeFrontLeftSwitch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Constants.kRobotStartingPose.getTranslation(), Rotation2d.fromDegrees(-60.0)));
            waypoints.add(new Pose2d(kLeftSwitchScoringPose, Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeFrontLeftToOuterCube(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kLeftSwitchScoringPose, Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(kOuterCubeIntakingPose, Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeOuterToLeftSwitch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kOuterCubeIntakingPose, Rotation2d.fromDegrees(-90.0)));
            waypoints.add(new Pose2d(kLeftSwitchScoringPose, Rotation2d.fromDegrees(-90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeFrontLeftToMiddleCube(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kLeftSwitchScoringPose, Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(kMiddleCubeIntakingPose, Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeMiddleToLeftSwitch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kMiddleCubeIntakingPose, Rotation2d.fromDegrees(-90.0)));
            waypoints.add(new Pose2d(kLeftSwitchScoringPose2, Rotation2d.fromDegrees(-90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeFrontLeftToBottomLeft(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kLeftSwitchScoringPose2, Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(kBottomLeftIntakingPose, Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeBottomLeftToSwitch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kBottomLeftIntakingPose, Rotation2d.fromDegrees(-90.0)));
            waypoints.add(new Pose2d(kLeftSwitchScoringPose2, Rotation2d.fromDegrees(-90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 3.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStraightPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(Constants.kRobotStartingPose);
            waypoints.add(Constants.kRobotStartingPose.transformBy(Pose2d.fromTranslation(new Translation2d(120.0, 0.0))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 96.0, 96.0, 24.0, kMaxVoltage, 60.0, 1);
        }
    }
    
}

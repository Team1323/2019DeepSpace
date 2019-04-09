package com.team1323.frc2019;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.team1323.frc2019.auto.SmartDashboardInteractions;
import com.team1323.frc2019.auto.SmartDashboardInteractions.Alliance;
import com.team1323.frc2019.subsystems.Swerve;
import com.team1323.frc2019.vision.GoalTracker;
import com.team1323.frc2019.vision.GoalTracker.TrackReport;
import com.team1323.frc2019.vision.ShooterAimingParameters;
import com.team1323.frc2019.vision.TargetInfo;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
	private static RobotState instance = new RobotState();
	public static RobotState getInstance(){
		return instance;
	}
	
	private static final int kObservationBufferSize = 100;
	
	private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
	private GoalTracker goal_tracker_;
	private Rotation2d camera_pitch_correction_;
    private Rotation2d camera_yaw_correction_;
    private double differential_height_;
    private double targetHeight = Constants.kDiskTargetHeight;
	private double distance_driven_;
    private ShooterAimingParameters cached_shooter_aiming_params_ = null;
    
    public final int minimumTargetQuantity = 3;
    private final int primaryTargetIndex = 2;
    private Translation2d lastKnownTargetPosition = new Translation2d();
    public Translation2d lastKnownTargetPosition(){ return lastKnownTargetPosition; }

    public double distanceToTarget(){
        return getLatestFieldToVehicle().getValue().getTranslation().distance(lastKnownTargetPosition);
    }

    Alliance alliance = SmartDashboardInteractions.STANDARD_CARPET_SIDE;
    public void setAlliance(Alliance a){
        alliance = a;
    }
    public Alliance getAlliance(){
        return alliance;
    }
    public boolean onStandardCarpet(){
        return alliance == SmartDashboardInteractions.STANDARD_CARPET_SIDE;
    }

    private boolean seesTarget = false;
    public boolean seesTarget(){
        return seesTarget;
    }
	
	private double angleToCube = 0;
	public double getAngleToCube(){
		return angleToCube;
	}
	public void setAngleToCube(double angle){
		angleToCube = angle;
	}
	
	public Translation2d getCubePosition(){
		List<TrackReport> reports = goal_tracker_.getTracks();
		if(!reports.isEmpty())
			return goal_tracker_.getTracks().get(0).field_to_goal;
		else
			return new Translation2d();
	}
	
	private static final Pose2d kVehicleToCamera = new Pose2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), Rotation2d.fromDegrees(0.0));
	
	private RobotState() {
        reset(0, new Pose2d());
    }
	
	/**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        goal_tracker_ = new GoalTracker();
        camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
        camera_yaw_correction_ = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
        differential_height_ = targetHeight - Constants.kCameraZOffset;
        distance_driven_ = 0.0;
    }
    
    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized void setVisionTargetHeight(double height){
        targetHeight = height;
        differential_height_ = height - Constants.kCameraZOffset;
    }

    public double getVisionTargetHeight(){
        return targetHeight;
    }
    
    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }
    
    public synchronized Pose2d getFieldToCamera(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(kVehicleToCamera);
    }

    public void setXTarget(double x, double error){
        goal_tracker_.setXTarget(x, error);
    }

    public void enableXTarget(boolean enable){
        goal_tracker_.enableXTarget(enable);
    }
    
    public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
        List<Translation2d> field_to_goals = new ArrayList<>();
        Pose2d field_to_camera = getFieldToCamera(timestamp);
        if (/*!(vision_update == null || vision_update.isEmpty())*/ vision_update.size() >= minimumTargetQuantity) {
            seesTarget = true;
            for (TargetInfo target : vision_update) {
                double ydeadband = target.getY();

                // Compensate for camera yaw
                double xyaw = target.getX() * camera_yaw_correction_.cos() + ydeadband * camera_yaw_correction_.sin();
                double yyaw = ydeadband * camera_yaw_correction_.cos() - target.getX() * camera_yaw_correction_.sin();
                double zyaw = target.getZ();

                // Compensate for camera pitch
                double xr = zyaw * camera_pitch_correction_.sin() + xyaw * camera_pitch_correction_.cos();
                double yr = yyaw;
                double zr = zyaw * camera_pitch_correction_.cos() - xyaw * camera_pitch_correction_.sin();
                
                // find intersection with the goal
                //if (zr < 0) {
                    double scaling = differential_height_ / zr;
                    double distance = Math.hypot(xr, yr) * scaling;
                    Rotation2d angle = new Rotation2d(xr, yr, true);
                    field_to_goals.add(field_to_camera
                            .transformBy(Pose2d
                                    .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
                            .getTranslation());
                //}
            }
        }else{
            seesTarget = false;
        }
        synchronized (this) {
            goal_tracker_.update(timestamp, field_to_goals);
        }
    }
    
    public synchronized Optional<ShooterAimingParameters> getCachedAimingParameters() {
        return cached_shooter_aiming_params_ == null ? Optional.empty() : Optional.of(cached_shooter_aiming_params_);
    }
    
    public synchronized Optional<ShooterAimingParameters> getAimingParameters() {
        List<TrackReport> reports = goal_tracker_.getTracks();
        if (reports.size() >= minimumTargetQuantity) {
            TrackReport report = reports.get(primaryTargetIndex);
            lastKnownTargetPosition = report.field_to_goal;
            Translation2d robot_to_goal = getLatestFieldToVehicle().getValue().getTranslation().inverse()
                    .translateBy(report.field_to_goal);
            Rotation2d targetOrientation1 = getTargetOrientation(reports.get(0).field_to_goal, reports.get(1).field_to_goal);
            Rotation2d targetOrientation2 = getTargetOrientation(reports.get(0).field_to_goal, reports.get(2).field_to_goal);
            Rotation2d targetOrientation3 = getTargetOrientation(reports.get(1).field_to_goal, reports.get(2).field_to_goal);

            Rotation2d targetOrientation = targetOrientation1;
            Rotation2d robot_to_goal_rotation = Rotation2d
                    .fromRadians(Math.atan2(robot_to_goal.y(), robot_to_goal.x()));
            SmartDashboard.putNumber("Vision Target Angle", targetOrientation.getDegrees());

            ShooterAimingParameters params = new ShooterAimingParameters(robot_to_goal.norm(), robot_to_goal_rotation,
                    report.latest_timestamp, report.stability, targetOrientation);
            cached_shooter_aiming_params_ = params;

            return Optional.of(params);
        } else {
            return Optional.empty();
        }
    }

    private Rotation2d getTargetOrientation(Translation2d target1, Translation2d target2){
        Translation2d targetDelta = target1.inverse().translateBy(target2);
        Rotation2d targetOrientation = Rotation2d.fromRadians(Math.atan2(targetDelta.y(), targetDelta.x())).rotateBy(Rotation2d.fromDegrees(-90.0));
        Rotation2d secondTargetOrientation = targetOrientation.rotateBy(Rotation2d.fromDegrees(180.0));
        if(Math.abs(targetOrientation.distance(getLatestFieldToVehicle().getValue().getRotation())) > Math.abs(secondTargetOrientation.distance(getLatestFieldToVehicle().getValue().getRotation()))){
            targetOrientation = secondTargetOrientation;
        }
        return targetOrientation;
    }

    public synchronized Optional<Pose2d> getRobotScoringPosition(Optional<ShooterAimingParameters> aimingParameters, Rotation2d orientation, Translation2d endTranslation){
        List<Pose2d> targetPositions = getCaptureTimeFieldToGoal();
		if(targetPositions.size() >= minimumTargetQuantity && aimingParameters.isPresent()){
            Translation2d targetPosition = targetPositions.get(primaryTargetIndex).getTranslation();
            SmartDashboard.putNumberArray("Path Pose", new double[]{targetPosition.x(), targetPosition.y(), aimingParameters.get().getTargetOrientation().getDegrees(), 0.0}); 
			Pose2d orientedTargetPosition = new Pose2d(targetPosition, orientation).transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
            Pose2d robotScoringPosition = orientedTargetPosition.transformBy(Pose2d.fromTranslation(endTranslation));
            
            return Optional.of(robotScoringPosition);
        }
        return Optional.empty();
    }

    public synchronized Optional<Pose2d> getOrientedTargetPosition(Optional<ShooterAimingParameters> aimingParameters){
        List<Pose2d> targetPositions = getCaptureTimeFieldToGoal();
		if(targetPositions.size() >= minimumTargetQuantity && aimingParameters.isPresent()){
            Translation2d targetPosition = targetPositions.get(primaryTargetIndex).getTranslation();
            SmartDashboard.putNumberArray("Path Pose", new double[]{targetPosition.x(), targetPosition.y(), aimingParameters.get().getTargetOrientation().getDegrees(), 0.0}); 
			Pose2d orientedTargetPosition = new Pose2d(targetPosition, aimingParameters.get().getTargetOrientation());
            
            return Optional.of(orientedTargetPosition);
        }
        return Optional.empty();
    }
    
    public synchronized void resetRobotPosition(Translation2d targetPosition){
    	List<TrackReport> reports = goal_tracker_.getTracks();
        if (reports.size() >= minimumTargetQuantity) {
            TrackReport report = reports.get(primaryTargetIndex);
            Translation2d robotFrameToFieldFrame = report.field_to_goal.inverse().translateBy(targetPosition);
            if(robotFrameToFieldFrame.norm() <= 5.0){
            	Swerve.getInstance().resetPosition(new Pose2d(Swerve.getInstance().getPose().getTranslation().translateBy(robotFrameToFieldFrame), Swerve.getInstance().getPose().getRotation()));
            	System.out.println("Coordinates corrected by " + robotFrameToFieldFrame.norm() + " inches");
            }else{
            	System.out.println("Coordinate correction too large: " + robotFrameToFieldFrame.norm());
            }
        }else{
        	System.out.println("Vision did not detect target");
        }
    }
    
    public synchronized List<Pose2d> getCaptureTimeFieldToGoal() {
        List<Pose2d> rv = new ArrayList<>();
        for (TrackReport report : goal_tracker_.getTracks()) {
            rv.add(Pose2d.fromTranslation(report.field_to_goal));
        }
        return rv;
    }

    public synchronized void clearVisionTargets(){
        goal_tracker_.clearTracks();
    }

    public synchronized void feignVisionTargets(){
        List<Translation2d> fakeTargets = Arrays.asList(
            new Translation2d(100.0, 50.0),
            new Translation2d(100.0, 58.0),
            new Translation2d(100.0, 54.0)
        );
        goal_tracker_.update(Timer.getFPGATimestamp(), fakeTargets);
    }
    
    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }
    
    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }
    
    public void outputToSmartDashboard(){
        SmartDashboard.putBoolean("Sees Target", seesTarget);

        /*List<Pose2d> targets = getCaptureTimeFieldToGoal();
        if(targets.size() >= minimumTargetQuantity){
            Translation2d targetPosition = targets.get(primaryTargetIndex).getTranslation();
            SmartDashboard.putNumberArray("Path Pose", new double[]{targetPosition.x(), targetPosition.y(), 0.0, 0.0}); 
        }*/

        /*Optional<ShooterAimingParameters> aim = getAimingParameters();
            if (aim.isPresent()) {
                SmartDashboard.putNumber("goal_range", aim.get().getRange());
            } else {
                SmartDashboard.putNumber("goal_range", 0.0);
            }*/

        if(Constants.kDebuggingOutput){
            List<Pose2d> poses = getCaptureTimeFieldToGoal();
            for (Pose2d pose : poses) {
                // Only output first goal
                SmartDashboard.putNumber("goal_pose_x", pose.getTranslation().x());
                SmartDashboard.putNumber("goal_pose_y", pose.getTranslation().y());

                break;
            }
            Optional<ShooterAimingParameters> aiming_params = /*getCachedAimingParameters();*/getAimingParameters();
            if (aiming_params.isPresent()) {
                SmartDashboard.putNumber("goal_range", aiming_params.get().getRange());
                SmartDashboard.putNumber("goal_theta", aiming_params.get().getRobotToGoal().getDegrees());
                getOrientedTargetPosition(aiming_params);
            } else {
                SmartDashboard.putNumber("goal_range", 0.0);
                SmartDashboard.putNumber("goal_theta", 0.0);
            }
        }
    }
}

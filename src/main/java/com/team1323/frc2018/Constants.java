package com.team1323.frc2018;

import java.util.Arrays;
import java.util.List;

import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Constants {
	public static final double kLooperDt = 0.02;
	
	public static final double kEpsilon = 0.0001;
	
	public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;
	public static final boolean kExtraNyooms = true;
	
	//Physical Robot Dimensions (including bumpers)
	public static final double kRobotWidth = 39.0;
	public static final double kRobotLength = 34.0;
	public static final double kRobotHalfWidth = kRobotWidth / 2.0;
	public static final double kRobotHalfLength = kRobotLength / 2.0;
	public static final double kRobotIntakeExtrusion = 11.0;//TODO update

	public static final double kBallRadius = 6.5;
	
	//Field Landmarks
	public static final Pose2d closeHatchPosition = new Pose2d(new Translation2d(48.0 + 166.57, 27.44 - 10.0), Rotation2d.fromDegrees(-30.0));
	public static final Pose2d farHatchPosition = new Pose2d(new Translation2d(229.13 + 14.71, 27.44 - 10.0), Rotation2d.fromDegrees(-150.0));
	public static final Pose2d humanLoaderPosition = new Pose2d(new Translation2d(0.0, 25.72), Rotation2d.fromDegrees(0.0));
	public static final Pose2d autoBallPosition = new Pose2d(new Translation2d(48.0 - 4.0 - kBallRadius, 97.0 - (3.0*kBallRadius)), Rotation2d.fromDegrees(-45.0));
	public static final Pose2d rocketPortPosition = new Pose2d(new Translation2d(229.13, 27.44), Rotation2d.fromDegrees(-90.0));

	public static final double kHatchTargetHeight = 28.5;
	
	public static final Pose2d kRobotStartingPose = new Pose2d(new Translation2d(48.0 + Constants.kRobotHalfLength, 97.0 + Constants.kRobotHalfWidth), Rotation2d.fromDegrees(0));
	
	//Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 18.5;
    public static final double kWheelbaseWidth  = 23.5;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants
    public static final double kCameraYOffset = kRobotHalfWidth - 16.0 - 3.0;
    public static final double kCameraXOffset = kRobotHalfLength - 8.0 - 3.0;
    public static final double kCameraZOffset = 20.0;
    public static final double kCameraYawAngleDegrees = 1.0;
    public static final double kCameraPitchAngleDegrees = -1.9;
    
    //Goal tracker constants
    public static double kMaxGoalTrackAge = 1.0;
    public static double kMaxTrackerDistance = 3.0;//18.0
    public static double kCameraFrameRate = 90.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
	public static double kTrackReportComparatorAgeWeight = 1.0;
	public static final double kDefaultCurveDistance = kRobotHalfLength + 36.0;
	public static final double kVisionUpdateDistance = kRobotHalfLength + 52.0;
	public static final double kVisionDistanceStep = kVisionUpdateDistance / 6.0;
	public static final double kClosestVisionDistance = 40.0;
    
    //Path following constants
    public static final double kPathLookaheadTime = 0.25;  // seconds to look ahead along the path for steering 0.4
	public static double kPathMinLookaheadDistance = 6.0;  // inches 24.0 (we've been using 3.0)
	public static void setLookaheadDistance(double distance){
		kPathMinLookaheadDistance = distance;
	}
    
    //Swerve Speed Constants
    public static final double kSwerveDriveMaxSpeed = 5432.0;
    public static final double kSwerveMaxSpeedInchesPerSecond = 12.5 * 12.0;
	public static final double kSwerveRotationMaxSpeed = 1250.0 * 0.8; //The 0.8 is to request a speed that is always achievable
	public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;
    
    //Swerve Module Wheel Offsets (Rotation encoder values when the wheels are facing 0 degrees)
	public static final int kFrontRightEncoderStartingPos = kIsUsingCompBot ? -3799 : -3614;
	public static final int kFrontLeftEncoderStartingPos = kIsUsingCompBot ? -198 : -2578;
	public static final int kRearLeftEncoderStartingPos = kIsUsingCompBot ? -2825 : -1504;
	public static final int kRearRightEncoderStartingPos = kIsUsingCompBot ? -2013 : -3209;
	
	//Swerve Module Positions (relative to the center of the drive base)
	public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength/2, kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength/2, -kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength/2, -kWheelbaseWidth/2);
	public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength/2, kWheelbaseWidth/2);
	
	public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
			kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);
	
	//Scrub Factors
	public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
	public static final double kXScrubFactor = 1.0 - (0.5665 / 13.4356);
	public static final double kYScrubFactor = 1.0 - (0.57527 / 20.4593);

	//Voltage-Velocity equation constants {m, b, x-intercept}
	//First set is the positive direction, second set is negative
	public static final double[][][] kVoltageVelocityEquations = new double[][][]{
		{{1.70, -4.39, 2.58}, {1.83, 5.23, -2.85}}, 
		{{1.59, -3.86, 2.42}, {1.43, 3.09, -2.16}}, 
		{{1.53, -3.66, 2.39}, {1.66, 4.15, -2.50}}, 
		{{1.84, -4.70, 2.56}, {1.85, 5.34, -2.89}}};
	
	//Swerve Odometry Constants
	public static final double kSwerveWheelDiameter = 3.93; //inches
	public static final double kSwerveDriveEncoderResolution = 4096.0;
	/** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
	public static final double kSwerveEncoderToWheelRatio = 10.0/9.0;
	public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
	public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
	
	//Elevator Constants
	public static final double kElevatorMaxSpeedHighGear = 535.6785 * 4096.0 / 600.0; //encoder units per 100 ms
	public static final double kElevatorMaxSpeedLowGear = 169.67 * 4096.0 / 600.0; //encoder units per 100 ms
	/** Pulse width position of the elevator encoder when it has fully descended. */
	public static final int kElevatorEncoderStartingPosition = 0;
	public static final double kElevatorTicksPerFoot = 11983.0 / 2.5989583; //determined empirically
	public static final double kElevatorHeightTolerance = 0.1; //feet
	public static final double kElevatorIntakingHeight = kIsUsingCompBot ? 0.2 : 0.125; //feet
	public static final double kElevatorSecondCubeHeight = 0.97 + 0.1;
	public static final double kElevatorHumanLoadHeight = 1.836 + 0.1;
	public static final double kElevatorSwitchHeight = 2.0; //feet
	public static final double kELevatorBalancedScaleHeight = 5.05; //feet
	public static final double kElevatorHighScaleHeight = 5.3;
	public static final double kElevatorLowScaleHeight = 4.3;
	public static final double kELevatorHangingHeight = 4.9;
	public static final double kElevatorMinHeight = 0.0; //feet
	public static final double kElevatorMaxHeight = 5.4; //feet
	public static final double kElevatorMaxCurrent = 50.0;//amps
	public static final int kELevatorCurrentLimit = 20;
	public static final double kElevatorMinimumHangingHeight = 0.795 + 0.08;
	public static final double kElevatorMaximumHangingHeight = 3.25;
	public static final double kElevatorHangingRampHeight = 3.452;
	public static final double kElevatorTippingCubeHeight = 0.57;
	public static final double kElevatorTeleopManualSpeed = 0.5;
	//0.905
	
	//Swerve Speed Constraint Treemap
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kSwerveSpeedTreeMap = new InterpolatingTreeMap<>();
	static{
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(-0.1), new InterpolatingDouble(1.0));
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(1.0));
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(kElevatorIntakingHeight), new InterpolatingDouble(1.0));
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(kElevatorMaxHeight), new InterpolatingDouble(0.5));
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(kElevatorMaxHeight + 0.2), new InterpolatingDouble(0.5));
	}
	
	//Wrist Constants
	public static final double kWristMaxSpeed = /*41.58 * 4096.0 / 600.0*/300.0; //encoder units per 100 ms
	public static final double kWristStartingAngle = 90.0;
	/** Pulse width position of the wrist encoder when the wrist is upright (at 90 degrees, parallel to the elevator). */
	public static final int kWristStartingEncoderPosition = kIsUsingCompBot ? 3249 : 559;
	/** The number of rotations the wrist encoder undergoes for every rotation of the wrist. */
	public static final double kWristEncoderToOutputRatio = 41.58 / 19.19;
	public static final double kWristAngleTolerance = 10.0; //degrees
	public static final double kWristMinControlAngle = -2.0; //degrees
	public static final double kWristMaxControlAngle = 100.0; //degrees
	public static final double kWristMinPhysicalAngle = -20.0;
	public static final double kWristMaxPhysicalAngle = 110.0;//95.192
	public static final double kWristIntakingAngle = kIsUsingCompBot ? 6.5 : 9.0;
	public static final double kWristPrimaryStowAngle = 85.0;
	public static final double kWristSecondaryStowAngle = 60.0;
	public static final double kWristHangingAngle = 90.0;
	public static final double kWristMaxStowHeight = 3.5; //height of the elevator
	public static final double kWristMaxCurrent = 40.0;//amps
	
	//Intake Constants
	public static final double kIntakeWeakEjectOutput = -0.4;
	public static final double kIntakeEjectOutput = kIsUsingCompBot ? -0.6 : -0.9;
	public static final double kIntakeStrongEjectOutput = -1.0;
	public static final double kIntakingOutput = 1.0;
	public static final double kIntakeWeakHoldingOutput = 1.25/12.0;
	public static final double kIntakeStrongHoldingOutput = 4.0/12.0;
	public static final double kIntakingResuckingOutput = 6.0/12.0;
	public static final double kIntakeRampRate = 0.25;
}

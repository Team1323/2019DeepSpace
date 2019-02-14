package com.team1323.frc2019;

import java.util.Arrays;
import java.util.List;

import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Constants {
	/*All distance measurements are in inches, unless otherwise noted.*/

	public static final double kLooperDt = 0.02;
	
	public static final double kEpsilon = 0.0001;
	
	public static final boolean kIsUsingCompBot = false;
	public static final boolean kIsUsingTractionWheels = true;

	public static final boolean kDebuggingOutput = true;
	
	//Physical Robot Dimensions (including bumpers)
	public static final double kRobotWidth = 33.0;
	public static final double kRobotLength = 33.0;
	public static final double kRobotHalfWidth = kRobotWidth / 2.0;
	public static final double kRobotHalfLength = kRobotLength / 2.0;
	public static final double kRobotIntakeExtrusion = 11.0;//TODO update

	public static final double kBallRadius = 6.5;
	
	//Field Landmarks
	public static final Pose2d closeHatchPosition = new Pose2d(new Translation2d(48.0 + 166.57, 27.44 - 10.0 - 162.0), Rotation2d.fromDegrees(-30.0));
	public static final Pose2d farHatchPosition = new Pose2d(new Translation2d(229.13 + 14.71, 27.44 - 10.0 - 162.0), Rotation2d.fromDegrees(-150.0));
	public static final Pose2d humanLoaderPosition = new Pose2d(new Translation2d(0.0, 25.72 - 162.0), Rotation2d.fromDegrees(0.0));
	public static final Pose2d autoBallPosition = new Pose2d(new Translation2d(48.0 - 4.0 - kBallRadius, 97.0 - (3.0*kBallRadius) - 162.0), Rotation2d.fromDegrees(-45.0));
	public static final Pose2d rocketPortPosition = new Pose2d(new Translation2d(229.13, 27.44 - 162.0), Rotation2d.fromDegrees(-90.0));

	public static final double kHatchTargetHeight = 28.5;
	
	public static final Pose2d kRobotLeftStartingPose = new Pose2d(new Translation2d(48.0 + kRobotHalfLength, 97.0 + kRobotHalfWidth - 162.0), Rotation2d.fromDegrees(0));
	public static final Pose2d kRobotRightStartingPose = new Pose2d(new Translation2d(48.0 + kRobotHalfLength, -(97.0 + kRobotHalfWidth - 162.0)), Rotation2d.fromDegrees(0));
	public static final Pose2d kRobotLeftRampExitPose = new Pose2d(new Translation2d(48.0 + kRobotHalfLength, -75.25 - kRobotHalfWidth), Rotation2d.fromDegrees(0));
	public static final Pose2d kRobotRightRampExitPose = new Pose2d(new Translation2d(48.0 + kRobotHalfLength, 75.25 + kRobotHalfWidth), Rotation2d.fromDegrees(0));
	
	//Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 21.0;
    public static final double kWheelbaseWidth  = 21.0;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants
    public static final double kCameraYOffset = 0.0;
    public static final double kCameraXOffset = (29.5 / 2.0) - 17.4375;
    public static final double kCameraZOffset = 39.906;
    public static final double kCameraYawAngleDegrees = 0.0;
    public static final double kCameraPitchAngleDegrees = -16.0;
    
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
	public static final double kPathMinLookaheadDistance = 6.0;  // inches 24.0 (we've been using 3.0)
    
    //Swerve Speed Constants
    public static final double kSwerveDriveMaxSpeed = 5432.0;
    public static final double kSwerveMaxSpeedInchesPerSecond = 12.5 * 12.0;
	public static final double kSwerveRotationMaxSpeed = 1250.0 * 0.8; //The 0.8 is to request a speed that is always achievable
	public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;
    
    //Swerve Module Wheel Offsets (Rotation encoder values when the wheels are facing 0 degrees)
	public static final int kFrontRightEncoderStartingPos = kIsUsingCompBot ? -3799 : 249 - 1024;
	public static final int kFrontLeftEncoderStartingPos = kIsUsingCompBot ? -198 : -2895 - 1024;
	public static final int kRearLeftEncoderStartingPos = kIsUsingCompBot ? -2825 : 636 - 1024;
	public static final int kRearRightEncoderStartingPos = kIsUsingCompBot ? -2013 : -1739 - 1024;
	
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
	public static final double kSwerveWheelDiameter = 4.0; //inches
	public static final double kSwerveDriveEncoderResolution = 4096.0;
	/** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
	public static final double kSwerveEncoderToWheelRatio = 10.0/9.0;
	public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
	public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
	
	//Elevator Constants
	public static final double kElevatorMaxSpeedHighGear = 541.18 * 4096.0 / 600.0; //encoder units per 100 ms
	/** Pulse width position of the elevator encoder when it has fully descended. */
	public static final int kElevatorEncoderStartingPosition = 0;
	public static final double kElevatorTicksPerInch = 6097.0 / 7.625; //determined empirically 5.12 inches before wrap
	public static final double kElevatorHeightTolerance = 1.0; //inches
	public static final double kElevatorDiskIntakeHeight = 2.5;
	public static final double kElevatorLowHatchHeight = 5.5;
	public static final double kElevatorMidHatchHeight = 35.0;
	public static final double kElevatorHighHatchHeight = 63.7;
	public static final double kElevatorBallIntakeHeight = 0.1;
	public static final double kElevatorLowBallHeight = 4.3;
	public static final double kElevatorMidBallHeight = 32.0;
	public static final double kElevatorHighBallHeight = 60.4;
	public static final double kElevatorMinHeight = 0.0; //inches
	public static final double kElevatorMaxHeight = 65.0; //inches
	public static final double kElevatorMaxCurrent = 50.0;//amps
	public static final int kELevatorCurrentLimit = 20;
	public static final double kElevatorMinimumHangingHeight = 0.795 + 0.08;
	public static final double kElevatorMaximumHangingHeight = 3.25;
	public static final double kElevatorHangingRampHeight = 3.452;
	public static final double kElevatorTippingCubeHeight = 0.57;
	public static final double kElevatorTeleopManualSpeed = 0.5;
	
	//Swerve Speed Constraint Treemap
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kSwerveSpeedTreeMap = new InterpolatingTreeMap<>();
	static{
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(-1.0), new InterpolatingDouble(1.0));
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(1.0));
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(kElevatorLowHatchHeight), new InterpolatingDouble(1.0));
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(kElevatorMaxHeight), new InterpolatingDouble(0.5));
		kSwerveSpeedTreeMap.put(new InterpolatingDouble(kElevatorMaxHeight + 2.0), new InterpolatingDouble(0.5));
	}
	
	//Wrist Constants
	public static final double kWristMaxSpeedHighGear = 600.0; //encoder units per 100 ms
	public static final double kWristMaxSpeedLowGear = 200.0;
	public static final double kWristStartingAngle = 0.0;
	/** Pulse width position of the wrist encoder when the wrist is upright (at 90 degrees, parallel to the elevator). */
	public static final int kWristStartingEncoderPosition = kIsUsingCompBot ? 3249 : 1179;
	/** The number of rotations the wrist encoder undergoes for every rotation of the wrist. */
	public static final double kWristEncoderToOutputRatio = 30.0 / 12.0; // 144 degrees before wrap
	public static final double kWristAngleTolerance = 10.0; //degrees
	public static final double kWristMinControlAngle = -80.0; //degrees
	public static final double kWristMaxControlAngle = 85.0; //degrees
	public static final double kWristMinPhysicalAngle = -40.0;
	public static final double kWristMaxPhysicalAngle = 90.0;//95.192
	public static final double kWristIntakingAngle = kIsUsingCompBot ? 0.0 : 0.0;
	public static final double kWristPrimaryStowAngle = 85.0;
	public static final double kWristHangingAngle = -71.5;
	public static final double kWristBallHoldingAngle = 38.0;
	public static final double kWristBallFeedingAngle = 60.5;
	public static final double kWristMaxCurrent = 40.0;//amps
	
	//Ball Intake Constants
	public static final double kIntakeWeakEjectOutput = -0.4;
	public static final double kIntakeEjectOutput = kIsUsingCompBot ? -0.6 : -0.9;
	public static final double kIntakeStrongEjectOutput = -1.0;
	public static final double kIntakingOutput = 1.0;
	public static final double kIntakeWeakHoldingOutput = 2.0/12.0;
	public static final double kIntakeStrongHoldingOutput = 4.0/12.0;
	public static final double kIntakingResuckingOutput = 6.0/12.0;
	public static final double kIntakeRampRate = 0.25;
	public static final double kIntakeClimbOutput = 6.0/12.0;

	//Ball Carriage Constants
	public static final double kBallCarriageEjectOutput = -0.5;
	public static final double kBallCarriageReceiveOutput = -0.6;

	//Disk Intake Constants
	public static final double kDiskIntakingOutput = 1.0;
	public static final double kDiskIntakeRampRate = 0.25;
	public static final double kDiskIntakeWeakEjectOutput = -0.375;
	public static final double kDiskIntakeStrongEjectOutput = -0.9;
	public static final double kDiskStrongHoldingOutput = 3.0;
	public static final double kDiskIntakingResuckingOutput = 6.0/12.0;

	//Jack Constants
	public static final double kJackMaxSpeed = 5000.0;
	public static final double kJackTicksPerInch = 24.9629356 * 4096.0 / 30.4444882;//1.219 inches before wrap
	public static final double kJackHeightTolerance = 1.0; //inches
	public static final int kJackStartingEncPosition = 3555;//2804;
	public static final double kJackStartingHeight = 0.0;
	public static final double kJackMaxPhysicalHeight = 0.2;
	public static final double kJackMinPhysicalHeight = -1.0;
	public static final double kJackMaxControlHeight = 0.0;
	public static final double kJackMinControlHeight = -24.5; //-31.0

	//Jack Height Treemap
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kJackHeightTreeMap = new InterpolatingTreeMap<>();
	static{
		kJackHeightTreeMap.put(new InterpolatingDouble(90.0), new InterpolatingDouble(kJackMaxControlHeight));
		kJackHeightTreeMap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(kJackMaxControlHeight));
		kJackHeightTreeMap.put(new InterpolatingDouble(kWristMinControlAngle), new InterpolatingDouble(kJackMinControlHeight));
		kJackHeightTreeMap.put(new InterpolatingDouble(kWristMinControlAngle - 10.0), new InterpolatingDouble(kJackMinControlHeight));
	}

}

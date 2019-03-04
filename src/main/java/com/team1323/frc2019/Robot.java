/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019;

import java.util.Arrays;

import com.team1323.frc2019.auto.AutoModeBase;
import com.team1323.frc2019.auto.AutoModeExecuter;
import com.team1323.frc2019.auto.SmartDashboardInteractions;
import com.team1323.frc2019.auto.modes.TwoCloseOneBallMode;
import com.team1323.frc2019.loops.LimelightProcessor;
import com.team1323.frc2019.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2019.loops.Looper;
import com.team1323.frc2019.loops.QuinticPathTransmitter;
import com.team1323.frc2019.loops.RobotStateEstimator;
import com.team1323.frc2019.subsystems.BallCarriage;
import com.team1323.frc2019.subsystems.BallIntake;
import com.team1323.frc2019.subsystems.DiskIntake;
import com.team1323.frc2019.subsystems.DiskScorer;
import com.team1323.frc2019.subsystems.Elevator;
import com.team1323.frc2019.subsystems.LEDs;
import com.team1323.frc2019.subsystems.Subsystem;
import com.team1323.frc2019.subsystems.SubsystemManager;
import com.team1323.frc2019.subsystems.Superstructure;
import com.team1323.frc2019.subsystems.Swerve;
import com.team1323.frc2019.subsystems.Wrist;
import com.team1323.frc2019.subsystems.requests.RequestList;
import com.team1323.io.SwitchController;
import com.team1323.io.Xbox;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.InputRamp;
import com.team1323.lib.util.Logger;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Swerve swerve;
	private Elevator elevator;
	private Wrist wrist;
	private BallIntake ballIntake;
	private BallCarriage ballCarriage;
	private DiskIntake diskIntake;
	private DiskScorer diskScorer;
	//private Jacks jacks;
	private LEDs leds;
	private Superstructure s;
	private SubsystemManager subsystems;

	private LimelightProcessor limelight;

	private InputRamp elevatorInput = new InputRamp(0.0, 0.05, 0.05);

	private AutoModeExecuter autoModeExecuter = null;
	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();
	private SmartDashboardInteractions smartDashboardInteractions = new SmartDashboardInteractions();

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();

	private double timestamp;

	private RobotState robotState = RobotState.getInstance();

	private DriverStation ds = DriverStation.getInstance();

	private Xbox driver, coDriver;
	private SwitchController switchController;
	private final boolean useSwitchController = false;
	private final boolean oneControllerMode = false;
	private boolean flickRotation = false;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		s = Superstructure.getInstance();
		swerve = Swerve.getInstance();
		elevator = Elevator.getInstance();
		wrist = Wrist.getInstance();
		ballIntake = BallIntake.getInstance();
		ballCarriage = BallCarriage.getInstance();
		diskIntake = DiskIntake.getInstance();
		diskScorer = DiskScorer.getInstance();
		//jacks = Jacks.getInstance();
		leds = LEDs.getInstance();
		subsystems = new SubsystemManager(
				Arrays.asList(swerve, elevator, wrist, ballIntake, ballCarriage, diskIntake, diskScorer, /*jacks,*/ leds, s));

		limelight = LimelightProcessor.getInstance();
		
		if (useSwitchController) {
			switchController = new SwitchController(2);
		}
		driver = new Xbox(0);
		coDriver = new Xbox(1);
		driver.setDeadband(0.0);
		coDriver.setDeadband(0.4);

		Logger.clearLog();

		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		enabledLooper.register(LimelightProcessor.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(LimelightProcessor.getInstance());
		subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);

		swerve.zeroSensors();
		// swerve.zeroSensors(new Pose2d());

		smartDashboardInteractions.initWithDefaults();

		generator.generateTrajectories();

		AutoModeBase auto = new TwoCloseOneBallMode(true);
		//qTransmitter.addPaths(auto.getPaths());
		System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));

	}

	public void allPeriodic() {
		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
		SmartDashboard.putBoolean("Enabled", ds.isEnabled());
		SmartDashboard.putNumber("Match time", ds.getMatchTime());
		for (Subsystem s : subsystems.getSubsystems()) {
			if (s.hasEmergency)
				leds.conformToState(LEDs.State.EMERGENCY);
		}
	}

	public void autoConfig() {
		swerve.zeroSensors();
		swerve.setNominalDriveOutput(1.5);
		swerve.requireModuleConfiguration();
		//swerve.set10VoltRotationMode(true);

		elevator.setCurrentLimit(15);
		elevator.configForAutoSpeed();

		s.enableCompressor(false);
	}

	public void teleopConfig() {
		s.enableCompressor(true);
		swerve.setNominalDriveOutput(0.0);
		swerve.set10VoltRotationMode(false);
		elevator.setCurrentLimit(40);
		elevator.configForTeleopSpeed();
		wrist.setHighGear(true);
		wrist.setAngle(Constants.kWristBallFeedingAngle);
	}

	@Override
	public void autonomousInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();

			autoConfig();

			disabledLooper.stop();
			enabledLooper.start();

			SmartDashboard.putBoolean("Auto", true);

			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode());
			autoModeExecuter.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		if(swerve.getState() == Swerve.ControlState.VISION){
			leds.conformToState(LEDs.State.TARGET_VISIBLE);
		}else{
			leds.conformToState(LEDs.State.ENABLED);
		}
		allPeriodic();
	}

	@Override
	public void teleopInit() {
		try {
			disabledLooper.stop();
			enabledLooper.start();
			teleopConfig();
			SmartDashboard.putBoolean("Auto", false);
			leds.conformToState(LEDs.State.ENABLED);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		try {
			driver.update();
			coDriver.update();

			if (useSwitchController) {
				switchController.update();
			}

			timestamp = Timer.getFPGATimestamp();

			if (oneControllerMode)
				oneControllerMode();
			else
				twoControllerMode();


			if(s.isClimbing()) {
				leds.conformToState(LEDs.State.CLIMBING);
			} else if (swerve.getState() == Swerve.ControlState.VISION){
				leds.conformToState(LEDs.State.TARGET_TRACKING);
			} else if(robotState.seesTarget()) {
				leds.conformToState(LEDs.State.TARGET_VISIBLE);
			} else if (diskScorer.hasDisk()) {
				leds.conformToState(LEDs.State.DISK_IN_PROBE);
			} else if (ballIntake.hasBall()) {
				leds.conformToState(LEDs.State.BALL_IN_INTAKE);
			} else if (ballCarriage.hasBall()) {
				leds.conformToState(LEDs.State.BALL_IN_CARRIAGE);
			} else if (diskIntake.hasDisk()) {
				leds.conformToState(LEDs.State.DISK_IN_INTAKE);
			} else {
				leds.conformToState(LEDs.State.ENABLED);
			}

			allPeriodic();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();
			enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();
			leds.conformToState(LEDs.State.DISABLED);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			allPeriodic();
			smartDashboardInteractions.output();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {
		allPeriodic();
	}

	private void twoControllerMode() {
		if (coDriver.backButton.isBeingPressed()) {
			s.neutralState();
		}

		double swerveYInput = driver.getX(Hand.kLeft);
		double swerveXInput = -driver.getY(Hand.kLeft);
		double swerveRotationInput = (flickRotation ? 0.0 : driver.getX(Hand.kRight));

		if (useSwitchController) {
			swerveYInput = switchController.getX(Hand.kLeft);
			swerveXInput = -switchController.getY(Hand.kLeft);
			swerveRotationInput = (flickRotation ? 0.0 : switchController.getX(Hand.kRight));

			if (switchController.plusButton.wasPressed()) {

			} else if (switchController.minusButton.wasPressed()) {
				swerve.temporarilyDisableHeadingController();
				swerve.zeroSensors(Constants.kRobotLeftStartingPose);
				swerve.resetAveragedDirection();
			}

			if (switchController.xButton.isBeingPressed())
				swerve.rotate(0);
			else if (switchController.aButton.isBeingPressed())
				swerve.rotate(90);
			else if (switchController.bButton.isBeingPressed())
				swerve.rotate(180);
			else if (switchController.yButton.isBeingPressed())
				swerve.rotate(270);
		}

		swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, false, driver.leftTrigger.isBeingPressed());

		if (driver.rightCenterClick.shortReleased()) {
			if (flickRotation) {
				driver.rumble(3, 1);
			} else {
				driver.rumble(1, 1);
			}
			flickRotation = !flickRotation;
		}

		if (flickRotation) {
			swerve.updateControllerDirection(new Translation2d(-driver.getY(Hand.kRight), driver.getX(Hand.kRight)));
			if (!Util.epsilonEquals(
					Util.placeInAppropriate0To360Scope(swerve.getTargetHeading(),
							swerve.averagedDirection.getDegrees()),
					swerve.getTargetHeading(), swerve.rotationDivision / 2.0)) {
				swerve.rotate(swerve.averagedDirection.getDegrees());
			}
		}
		
		if (driver.bButton.isBeingPressed())
			swerve.rotate(90);
		else if (driver.aButton.isBeingPressed())
			swerve.rotate(180);
		else if (driver.xButton.isBeingPressed())
			swerve.rotate(270);
		else if (driver.leftBumper.shortReleased())
			swerve.rotate(-25);
		else if(driver.leftBumper.longPressed())
			swerve.rotate(-150.0);
		else if (driver.rightBumper.shortReleased())
			swerve.rotate(25);
		else if(driver.rightBumper.longPressed())
			swerve.rotate(150.0);
		if (driver.backButton.shortReleased() || driver.backButton.longPressed()) {
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotLeftStartingPose);
			swerve.resetAveragedDirection();
		} else if (driver.POV0.shortReleased()) {
			/*swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotLeftStartingPose);
			swerve.requireModuleConfiguration();
			// swerve.setTrajectory(generator.getTrajectorySet().startToCloseHatch.get(true),
			// -30.0, 1.0);
			swerve.setTrajectory(generator.getTrajectorySet().straightPath, 0.0, 1.0);*/
			// swerve.setVelocity(new Rotation2d(), 24.0);
		} else if (driver.startButton.shortReleased()) {
			limelight.setPipeline(Pipeline.CLOSEST);
			s.humanLoaderRetrievingState();
		} /*else if (driver.leftBumper.isBeingPressed()) {
			swerve.setVelocity(new Rotation2d(), 24.0);
		} else if (swerve.getState() == Swerve.ControlState.VELOCITY) {
			swerve.setState(Swerve.ControlState.MANUAL);
		}*/

		s.sendManualInput(-coDriver.getY(Hand.kLeft), -coDriver.getY(Hand.kRight), /*-coDriver.getY(Hand.kLeft)*/0.0);

		////// Official Controls //////

		if (coDriver.startButton.shortReleased()) {
			s.diskReceivingState();
		} else if(coDriver.startButton.longPressed()){
			limelight.setPipeline(Pipeline.CLOSEST);
			s.humanLoaderRetrievingState();
		} else if (coDriver.rightBumper.shortReleased()) {
			s.diskIntakingState();
		} else if (coDriver.leftBumper.shortReleased()) {
			diskIntake.conformToState(DiskIntake.State.EJECTING);
		} else if (coDriver.leftBumper.longPressed()) {
			diskIntake.conformToState(DiskIntake.State.OFF);
		} else if (coDriver.rightTrigger.shortReleased() || driver.rightTrigger.shortReleased()) {
			ballCarriage.conformToState(BallCarriage.State.EJECTING);
		} else if(coDriver.leftTrigger.shortReleased() || driver.yButton.shortReleased()){
			diskScorer.conformToState(DiskScorer.State.SCORING);
		} else if (coDriver.leftTrigger.longPressed()) {
			diskScorer.conformToState(DiskScorer.State.RECEIVING);
		} else if (coDriver.aButton.wasActivated()) {
			s.ballIntakingState();
		} else if (coDriver.aButton.wasReleased()) {
			s.fullBallCycleState();
		} else if (coDriver.xButton.shortReleased()) {
			if(diskScorer.isExtended()){
				//limelight.setPipeline(Pipeline.LOWEST);
				//s.diskTrackingState(Constants.kElevatorMidHatchHeight);
				elevator.setTargetHeight(Constants.kElevatorMidHatchHeight);
			}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
				//limelight.setPipeline(Pipeline.HIGHEST);
				//s.ballTrackingState(Constants.kElevatorMidBallHeight);
				elevator.setTargetHeight(Constants.kElevatorMidBallHeight);
			}
		} else if (coDriver.xButton.longPressed()) {
			if(diskScorer.isExtended()){
				s.diskScoringState(Constants.kElevatorMidHatchHeight);
			}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
				s.ballScoringState(Constants.kElevatorMidBallHeight);
			}
		} else if (coDriver.yButton.shortReleased()) {
			if(diskScorer.isExtended()){
				//limelight.setPipeline(Pipeline.LOWEST);
				//s.diskTrackingState(Constants.kElevatorHighHatchHeight);
				elevator.setTargetHeight(Constants.kElevatorHighHatchHeight);
			}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
				//limelight.setPipeline(Pipeline.HIGHEST);
				//s.ballTrackingState(Constants.kElevatorHighBallHeight);
				elevator.setTargetHeight(Constants.kElevatorHighBallHeight);
			}
		} else if (coDriver.yButton.longPressed()) {
			if(diskScorer.isExtended()){
				s.diskScoringState(Constants.kElevatorHighHatchHeight);
			}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
				s.ballScoringState(Constants.kElevatorHighBallHeight);
			}
		} else if (coDriver.bButton.shortReleased()) {
			if(diskScorer.isExtended()){
				limelight.setPipeline(Pipeline.LOWEST);
				s.diskTrackingState(Constants.kElevatorLowHatchHeight);
			}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
				limelight.setPipeline(Pipeline.HIGHEST);
				s.ballTrackingState(Constants.kElevatorLowBallHeight);
			}
		} else if (coDriver.bButton.longPressed()) {
			if(diskScorer.isExtended()){
				s.diskScoringState(Constants.kElevatorLowHatchHeight);
			}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
				s.ballScoringState(Constants.kElevatorLowBallHeight);
			}
		} else if (coDriver.rightCenterClick.shortReleased()) {
			s.request(new RequestList(Arrays.asList(
				ballCarriage.stateRequest(BallCarriage.State.RECEIVING),
				ballCarriage.waitForBallRequest(),
				ballCarriage.stateRequest(BallCarriage.State.SUCKING)), false));
		} else if (coDriver.leftCenterClick.shortReleased()) {
			ballIntake.conformToState(BallIntake.State.EJECTING);
		} else if (coDriver.POV0.shortReleased()) {
			s.climbingState();
		} else if (coDriver.POV180.shortReleased()) {
			s.postClimbingState();
		}

		if (diskScorer.needsToNotifyDrivers()) {
			driver.rumble(1.0, 1.0);
		}

		/*
		 * 
		 * if(coDriver.startButton.longPressed()){ elevator.setManualSpeed(0.75);
		 * elevator.enableLimits(false); coDriver.rumble(1.0, 1.0); }else
		 * if(!s.elevator.limitsEnabled() && coDriver.startButton.longReleased()){
		 * elevator.zeroSensors(); elevator.enableLimits(true);
		 * elevator.setManualSpeed(Constants.kElevatorTeleopManualSpeed);
		 * elevator.lockHeight(); }
		 */
	}

	private void oneControllerMode() {

	}
}

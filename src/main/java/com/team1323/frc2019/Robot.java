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
import com.team1323.frc2019.auto.modes.FarCloseBallMode;
import com.team1323.frc2019.loops.LimelightProcessor;
import com.team1323.frc2019.loops.Looper;
import com.team1323.frc2019.loops.QuinticPathTransmitter;
import com.team1323.frc2019.loops.RobotStateEstimator;
import com.team1323.frc2019.subsystems.BallIntake;
import com.team1323.frc2019.subsystems.DiskIntake;
import com.team1323.frc2019.subsystems.Elevator;
import com.team1323.frc2019.subsystems.Jacks;
import com.team1323.frc2019.subsystems.Probe;
import com.team1323.frc2019.subsystems.SubsystemManager;
import com.team1323.frc2019.subsystems.Superstructure;
import com.team1323.frc2019.subsystems.Swerve;
import com.team1323.frc2019.subsystems.Wrist;
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
	private DiskIntake diskIntake;
	private Probe probe;
	private Jacks jacks;
	private Superstructure superstructure;
	private SubsystemManager subsystems;
	
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
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		swerve = Swerve.getInstance();
		elevator = Elevator.getInstance();
		wrist = Wrist.getInstance();
		ballIntake = BallIntake.getInstance();
		diskIntake = DiskIntake.getInstance();
		probe = Probe.getInstance();
		jacks = Jacks.getInstance();
		subsystems = new SubsystemManager(
			Arrays.asList(swerve, elevator, wrist,
				ballIntake, diskIntake, probe, jacks));
		
		if(useSwitchController){
			switchController = new SwitchController(2);
		}
		driver = new Xbox(0);
		coDriver = new Xbox(1);
		driver.setDeadband(0.0);
		coDriver.setDeadband(0.4);
		
		Logger.clearLog();

		subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);
		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		enabledLooper.register(LimelightProcessor.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(LimelightProcessor.getInstance());

		swerve.zeroSensors();
		
		smartDashboardInteractions.initWithDefaults();
		
		generator.generateTrajectories();		

		AutoModeBase auto = new FarCloseBallMode(true);

		//qTransmitter.addPaths(auto.getPaths());
		System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));
	}
	
	public void allPeriodic(){
		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
		SmartDashboard.putBoolean("Enabled", ds.isEnabled());
		SmartDashboard.putNumber("Match time", ds.getMatchTime());
	}

	public void autoConfig(){
		swerve.zeroSensors();
		swerve.setNominalDriveOutput(1.5);
		swerve.requireModuleConfiguration();
		swerve.set10VoltRotationMode(true);

		superstructure.elevator.setCurrentLimit(15);
		superstructure.elevator.configForAutoSpeed();
		
		superstructure.enableCompressor(false);
	}

	public void teleopConfig(){
		superstructure.enableCompressor(true);
		swerve.setNominalDriveOutput(0.0);
		swerve.set10VoltRotationMode(false);
		superstructure.elevator.setCurrentLimit(15);
		superstructure.elevator.configForTeleopSpeed();
	}
	
	@Override
	public void autonomousInit() {
		try{
			if(autoModeExecuter != null)
				autoModeExecuter.stop();

			autoConfig();
			
			disabledLooper.stop();
			enabledLooper.start();
			
			SmartDashboard.putBoolean("Auto", true);
			
			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode());
			autoModeExecuter.start();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		allPeriodic();
	}
	
	@Override
	public void teleopInit(){
		try{
			disabledLooper.stop();
			enabledLooper.start();
			teleopConfig();
			SmartDashboard.putBoolean("Auto", false);

			superstructure.request(wrist.angleRequest(95.0));
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		try{
			driver.update();
			//coDriver.update();

			if(useSwitchController){
				switchController.update();
			}

			timestamp = Timer.getFPGATimestamp();
			
			if(oneControllerMode) oneControllerMode();
			else twoControllerMode();
			
			allPeriodic();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void disabledInit(){
		try{
			if(autoModeExecuter != null)
				autoModeExecuter.stop();
			enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void disabledPeriodic(){
		try{
			allPeriodic();
			smartDashboardInteractions.output();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void testInit(){
		
	}
	
	@Override
	public void testPeriodic() {
		allPeriodic();
	}

	private void twoControllerMode(){
		if(coDriver.backButton.isBeingPressed()){
		}
		
		double swerveYInput = driver.getX(Hand.kLeft);
		double swerveXInput = -driver.getY(Hand.kLeft);
		double swerveRotationInput = (flickRotation ? 0.0 : driver.getX(Hand.kRight));

		if(useSwitchController){
			swerveYInput = switchController.getX(Hand.kLeft);
			swerveXInput = -switchController.getY(Hand.kLeft);
			swerveRotationInput = (flickRotation ? 0.0 : switchController.getX(Hand.kRight));

			if(switchController.plusButton.wasPressed()){
				swerve.resetVisionUpdates();
				swerve.setVisionTrajectory();
			}else if(switchController.minusButton.wasPressed()){
				swerve.temporarilyDisableHeadingController();
				swerve.zeroSensors(Constants.kRobotLeftStartingPose);
				swerve.resetAveragedDirection();
			}

			if(switchController.xButton.isBeingPressed())
				swerve.rotate(0);
			else if(switchController.aButton.isBeingPressed())
				swerve.rotate(90);
			else if(switchController.bButton.isBeingPressed())
				swerve.rotate(180);
			else if(switchController.yButton.isBeingPressed())
				swerve.rotate(270);
		}
		
		swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, false, driver.leftTrigger.isBeingPressed());

		if(driver.rightCenterClick.wasPressed()){
			if(flickRotation){
				driver.rumble(3, 1);
			}else{
				driver.rumble(1, 1);
			}
			flickRotation = !flickRotation;
		}

		if(flickRotation){
			swerve.updateControllerDirection(new Translation2d(-driver.getY(Hand.kRight), driver.getX(Hand.kRight)));
			if(!Util.epsilonEquals(Util.placeInAppropriate0To360Scope(swerve.getTargetHeading(), swerve.averagedDirection.getDegrees()), swerve.getTargetHeading(), swerve.rotationDivision / 2.0)){
				swerve.rotate(swerve.averagedDirection.getDegrees());
			}
		}

		if(driver.yButton.isBeingPressed())
			swerve.rotate(0);
		else if(driver.bButton.isBeingPressed())
			swerve.rotate(90);
		else if(driver.aButton.isBeingPressed())
			swerve.rotate(180);
		else if(driver.xButton.isBeingPressed())
			swerve.rotate(270);
		else if(driver.leftCenterClick.isBeingPressed())
			swerve.rotate(-135);
		else if(driver.rightBumper.isBeingPressed())
			swerve.rotate(25);
		if(driver.backButton.wasPressed() || driver.backButton.longPressed()){
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotLeftStartingPose);
			swerve.resetAveragedDirection();
		}else if(driver.rightTrigger.wasPressed()){
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotLeftStartingPose);
			swerve.requireModuleConfiguration();
			swerve.setTrajectory(generator.getTrajectorySet().startToCloseHatch.get(true), -30.0, 1.0);
			//swerve.setVelocity(new Rotation2d(), 24.0);
		}else if(driver.startButton.wasPressed()){
			swerve.resetVisionUpdates();
			swerve.setVisionTrajectory();
		}

		superstructure.sendManualInput(-coDriver.getY(Hand.kRight), elevatorInput.update(-coDriver.getY(Hand.kLeft), timestamp));
		
		if(coDriver.startButton.longPressed()){
			elevator.setManualSpeed(0.75);//0.25
			superstructure.elevator.enableLimits(false);
			coDriver.rumble(1.0, 1.0);
		}else if(!superstructure.elevator.limitsEnabled() && coDriver.startButton.longReleased()){
			superstructure.elevator.zeroSensors();
			superstructure.elevator.enableLimits(true);
			elevator.setManualSpeed(Constants.kElevatorTeleopManualSpeed);
			elevator.lockHeight();
		}
	}

	private void oneControllerMode(){
		
	}
}

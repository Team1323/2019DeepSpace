/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019;

import com.team1323.frc2019.auto.AutoModeBase;
import com.team1323.frc2019.auto.AutoModeExecuter;
import com.team1323.frc2019.auto.SmartDashboardInteractions;
import com.team1323.frc2019.auto.modes.CloseFarBallMode;
import com.team1323.frc2019.loops.LimelightProcessor;
import com.team1323.frc2019.loops.Looper;
import com.team1323.frc2019.loops.QuinticPathTransmitter;
import com.team1323.frc2019.loops.RobotStateEstimator;
import com.team1323.frc2019.subsystems.LEDs;
import com.team1323.frc2019.subsystems.SubsystemManager;
import com.team1323.frc2019.subsystems.Superstructure;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.Logger;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Superstructure s;
	private SubsystemManager subsystems;
	private LEDs leds;

	private AutoModeExecuter autoModeExecuter = null;
	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();
	private SmartDashboardInteractions smartDashboardInteractions = new SmartDashboardInteractions();

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();

	private RobotState robotState = RobotState.getInstance();

	private DriverStation ds = DriverStation.getInstance();

	private DriverControls driverControls;
	

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		driverControls = DriverControls.getInstance();

		s = Superstructure.getInstance();
		subsystems = driverControls.getSubsystems();
		leds = LEDs.getInstance();


		Logger.clearLog();

		enabledLooper.register(driverControls);
		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		enabledLooper.register(LimelightProcessor.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(LimelightProcessor.getInstance());
		subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);

		s.swerve.zeroSensors();
		// swerve.zeroSensors(new Pose2d());

		robotState.feignVisionTargets();
		s.swerve.startTracking(Constants.kDiskTargetHeight, new Translation2d(-6.0, 0.0), true, new Rotation2d());
		s.swerve.stop();

		smartDashboardInteractions.initWithDefaults();

		generator.generateTrajectories();

		AutoModeBase auto = new CloseFarBallMode(true);
		qTransmitter.addPaths(auto.getPaths());
		System.out.println("Total path time: " + qTransmitter.getTotalPathTime(auto.getPaths()));

	}

	@Override
	public void robotPeriodic() {
		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
		//Pigeon.getInstance().outputToSmartDashboard();
		SmartDashboard.putBoolean("Enabled", ds.isEnabled());
		SmartDashboard.putNumber("Match time", ds.getMatchTime());
	}

	@Override
	public void autonomousInit() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();

			driverControls.setAutoMode(true);

			disabledLooper.stop();
			enabledLooper.start();

			SmartDashboard.putBoolean("Auto", true);

			robotState.setAlliance(smartDashboardInteractions.getSelectedAlliance());
			s.swerve.setCarpetDirection(robotState.onStandardCarpet());

			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode());
			autoModeExecuter.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopInit() {
		try {
			driverControls.setAutoMode(false);
			disabledLooper.stop();
			enabledLooper.start();
			SmartDashboard.putBoolean("Auto", false);
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
			
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			smartDashboardInteractions.output();
			if(subsystems.haveEmergency())
				leds.conformToState(LEDs.State.EMERGENCY);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {

	}

}

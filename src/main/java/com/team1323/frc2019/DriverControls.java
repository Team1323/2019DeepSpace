/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019;

import java.util.Arrays;

import com.team1323.frc2019.loops.LimelightProcessor;
import com.team1323.frc2019.loops.LimelightProcessor.Pipeline;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.BallCarriage;
import com.team1323.frc2019.subsystems.BallIntake;
import com.team1323.frc2019.subsystems.DiskIntake;
import com.team1323.frc2019.subsystems.DiskScorer;
import com.team1323.frc2019.subsystems.Elevator;
import com.team1323.frc2019.subsystems.Jacks;
import com.team1323.frc2019.subsystems.LEDs;
import com.team1323.frc2019.subsystems.SubsystemManager;
import com.team1323.frc2019.subsystems.Superstructure;
import com.team1323.frc2019.subsystems.Swerve;
import com.team1323.frc2019.subsystems.Wrist;
import com.team1323.io.Xbox;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * A class to assign controller inputs to robot actions
 */
public class DriverControls implements Loop {

    private static DriverControls instance = null;

    public static DriverControls getInstance() {
        if (instance == null)
            instance = new DriverControls();
        return instance;
    }

    Xbox driver, coDriver;

    private Swerve swerve;
    private Elevator elevator;
    private Wrist wrist;
	private BallIntake ballIntake;
	private BallCarriage ballCarriage;
	private DiskIntake diskIntake;
	private DiskScorer diskScorer;
    private Jacks jacks;
    private LEDs leds;
    private Superstructure s;

    private SubsystemManager subsystems;
    public SubsystemManager getSubsystems(){ return subsystems; }

    private RobotState robotState;

    private LimelightProcessor limelight;

    private final boolean oneControllerMode = false;
	private boolean flickRotation = false;
    private boolean robotCentric = false;
    
    private boolean inAuto = true;
    public void setAutoMode(boolean auto) {
        inAuto = auto;
    }

    public DriverControls() {
        driver = new Xbox(0);
        coDriver = new Xbox(1);
        driver.setDeadband(0.0);
		coDriver.setDeadband(0.4);
		coDriver.rightBumper.setLongPressDuration(1.0);

        swerve = Swerve.getInstance();
		elevator = Elevator.getInstance();
		wrist = Wrist.getInstance();
		ballIntake = BallIntake.getInstance();
		ballCarriage = BallCarriage.getInstance();
		diskIntake = DiskIntake.getInstance();
		diskScorer = DiskScorer.getInstance();
        jacks = Jacks.getInstance();
        leds = LEDs.getInstance();
        s = Superstructure.getInstance();

        subsystems = new SubsystemManager(
				Arrays.asList(swerve, elevator, wrist, ballIntake, ballCarriage, diskIntake, diskScorer, jacks, leds, s));

        robotState = RobotState.getInstance();

        limelight = LimelightProcessor.getInstance();
    }

    @Override
    public void onStart(double timestamp) {
        if(inAuto) {
            swerve.zeroSensors();
            swerve.setNominalDriveOutput(0.0);
            swerve.requireModuleConfiguration();
            //swerve.set10VoltRotationMode(true);

            elevator.setCurrentLimit(40);
            elevator.configForAutoSpeed();

		    s.enableCompressor(false);
        } else {
            s.enableCompressor(true);
            swerve.setNominalDriveOutput(0.0);
            swerve.set10VoltRotationMode(false);
            elevator.setCurrentLimit(40);
            elevator.configForTeleopSpeed();
            wrist.setHighGear(true);
            wrist.setAngle(Constants.kWristBallFeedingAngle);
            if(diskScorer.hasDisk())
                diskScorer.conformToState(DiskScorer.State.HOLDING);
                
            robotState.enableXTarget(false);
            leds.conformToState(LEDs.State.ENABLED);
        }
    }

    @Override
    public void onLoop(double timestamp) {
        if(inAuto) {
            if(s.swerve.getState() == Swerve.ControlState.VISION_TRAJECTORY){
                leds.conformToState(LEDs.State.TARGET_VISIBLE);
            }else{
                leds.conformToState(LEDs.State.ENABLED);
            }
        } else {
            driver.update();
            coDriver.update();
            if(oneControllerMode) oneControllerMode();
            else twoControllerMode();
        }
    }

    @Override
    public void onStop(double timestamp) {
        subsystems.stop();
        leds.conformToState(LEDs.State.RAINBOW);
    }

    private void twoControllerMode() {
        if (coDriver.backButton.isBeingPressed()) {
			s.neutralState();
		}

		double swerveYInput = driver.getX(Hand.kLeft);
		double swerveXInput = -driver.getY(Hand.kLeft);
		double swerveRotationInput = (flickRotation ? 0.0 : driver.getX(Hand.kRight));

		swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, robotCentric, driver.leftTrigger.isBeingPressed());

		if (driver.rightCenterClick.shortReleased()) {
			/*if (flickRotation) {
				driver.rumble(3, 1);
			} else {
				driver.rumble(1, 1);
			}
			flickRotation = !flickRotation;*/
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
			swerve.rotate(-24);
		else if(driver.leftBumper.longPressed())
			swerve.rotate(-151.0);
		else if (driver.rightBumper.shortReleased())
			swerve.rotate(24);
		else if(driver.rightBumper.longPressed())
			swerve.rotate(151.0);
		else if(driver.POV0.isBeingPressed())
			swerve.rotate(0.0);

		if (driver.backButton.shortReleased() || driver.backButton.longPressed()) {
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotLeftStartingPose);
			swerve.resetAveragedDirection();
		} else if (driver.startButton.shortReleased()) {
			if(!swerve.isTracking()){
				diskScorer.loseDisk();
				limelight.setPipeline(Pipeline.CLOSEST);
				s.humanLoaderRetrievingState();
			}
		} /*else if (driver.leftBumper.isBeingPressed()) {
			swerve.setVelocity(new Rotation2d(), 24.0);
		} else if (swerve.getState() == Swerve.ControlState.VELOCITY) {
			swerve.setState(Swerve.ControlState.MANUAL);
		}*/

		//s.sendManualInput(-coDriver.getY(Hand.kLeft), -coDriver.getY(Hand.kRight), /*-coDriver.getY(Hand.kLeft)*/0.0);

		double leftStick = coDriver.getY(Hand.kLeft);
		double rightStick = coDriver.getY(Hand.kRight);

		if(Math.abs(leftStick) != 0){
			wrist.setOpenLoop(-leftStick);
		}else if(wrist.isOpenLoop()){
			wrist.lockAngle();
		}

		if(Math.abs(rightStick) != 0){
			elevator.setOpenLoop(-rightStick);
		}else if(elevator.isOpenLoop()){
			elevator.lockHeight();
		}

		////// Official Controls //////

		if(!s.isClimbing()){
			if (coDriver.startButton.shortReleased()) {
				if(coDriver.leftTrigger.isBeingPressed()){
					if(!swerve.isTracking()){
						diskScorer.loseDisk();
						limelight.setPipeline(Pipeline.CLOSEST);
						s.humanLoaderRetrievingState();
					}
				}else{
					s.diskReceivingState();
				}
			} else if(coDriver.rightTrigger.shortReleased()){
				s.ballScoringState(Constants.kElevatorBallCargoShipHeight);
			} else if (coDriver.rightBumper.shortReleased()) {
				s.diskIntakingState();
			} else if (driver.rightTrigger.shortReleased()) {
				diskScorer.conformToState(DiskScorer.State.STOWED);
				ballCarriage.conformToState(BallCarriage.State.EJECTING);
			} else if(driver.rightTrigger.longPressed()){
				diskScorer.conformToState(DiskScorer.State.STOWED);
				ballCarriage.conformToState(BallCarriage.State.EJECTING, Constants.kBallCarriageWeakEjectOutput);
			} else if(driver.yButton.shortReleased()){
				//diskScorer.conformToState(DiskScorer.State.SCORING, Constants.kDiskEjectTreemap.getInterpolated(new InterpolatingDouble(elevator.getHeight())).value);
				s.diskScoringState();
				robotCentric = false;
			} else if (coDriver.aButton.wasActivated()) {
				diskScorer.checkForDisk();
				s.ballIntakingState();
			} else if (coDriver.aButton.wasReleased()) {
				s.fullBallCycleState();
			} else if (coDriver.leftTrigger.wasActivated()) {
				if(!swerve.isTracking()){
					if(elevator.hasReachedTargetHeight()){
						elevator.setTargetHeight(elevator.nearestVisionHeight(diskScorer.hasDisk() ? Constants.kElevatorDiskVisibleRanges : Constants.kElevatorBallVisibleRanges));
					}else{
						elevator.setTargetHeight(elevator.nearestVisionHeight(elevator.getTargetHeight(), diskScorer.hasDisk() ? Constants.kElevatorDiskVisibleRanges : Constants.kElevatorBallVisibleRanges));
					}
				}
			} else if (coDriver.xButton.wasActivated()) {
				if(coDriver.leftTrigger.isBeingPressed()){
					if(!swerve.isTracking()){
						if(diskScorer.hasDisk() || diskScorer.isExtended()){
							limelight.setPipeline(Pipeline.LOWEST);
							s.diskTrackingState(Constants.kElevatorMidHatchHeight);
						}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
							limelight.setPipeline(Pipeline.HIGHEST);
							s.ballTrackingState(Constants.kElevatorMidBallHeight);
						}
					}
				}else{
					if(diskScorer.hasDisk() || diskScorer.isExtended()){
						s.diskScoringState(Constants.kElevatorMidHatchHeight, false);
					}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
						s.ballScoringState(Constants.kElevatorMidBallHeight);
					}
				}
			} else if (coDriver.yButton.wasActivated()) {
				if(coDriver.leftTrigger.isBeingPressed()){
					if(!swerve.isTracking()){
						if(diskScorer.hasDisk() || diskScorer.isExtended()){
							limelight.setPipeline(Pipeline.LOWEST);
							s.diskTrackingState(Constants.kElevatorHighHatchHeight);
						}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
							limelight.setPipeline(Pipeline.HIGHEST);
							s.ballTrackingState(Constants.kElevatorHighBallHeight);
						}
					}
				}else{
					if(diskScorer.hasDisk() || diskScorer.isExtended()){
						s.diskScoringState(Constants.kElevatorHighHatchHeight, false);
					}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
						s.ballScoringState(Constants.kElevatorHighBallHeight);
					}
				}
			} else if (coDriver.bButton.shortReleased()) {
				if(coDriver.leftTrigger.isBeingPressed()){
					if(!swerve.isTracking()){
						if(diskScorer.hasDisk() || diskScorer.isExtended()){
							limelight.setPipeline(Pipeline.LOWEST);
							s.diskTrackingState(Constants.kElevatorLowHatchHeight);
						}else if(ballCarriage.getState() != BallCarriage.State.RECEIVING){
							limelight.setPipeline(Pipeline.HIGHEST);
							s.ballTrackingState(Constants.kElevatorLowBallHeight);
						}
					}
				}else{
					if(diskScorer.hasDisk() || diskScorer.isExtended()){
						s.diskScoringState(Constants.kElevatorLowHatchHeight, false);
					}else{
						s.ballScoringState(Constants.kElevatorLowBallHeight);
					}
				}
			} else if (coDriver.bButton.longPressed()) {
				s.diskScoringState(12.5, false);
			} else if (coDriver.rightBumper.longPressed()) {
				diskIntake.conformToState(DiskIntake.State.EJECTING);
			} else if (coDriver.leftCenterClick.shortReleased()) {
				ballIntake.conformToState(BallIntake.State.EJECTING);
			} else if(driver.leftCenterClick.shortReleased()){
				limelight.setPipeline(Pipeline.LOWEST);
				s.diskTrackingState();
			} else if(driver.POV180.wasActivated()){
				swerve.setState(Swerve.ControlState.MANUAL);
				robotCentric = false;
			} else if (driver.POV270.wasActivated()) {
				robotCentric = !robotCentric;
				if(robotCentric){
					driver.rumble(1.0, 1.0);
				}
			}
		}else{
			
		}

		if (coDriver.POV0.shortReleased() && !jacks.isAtHeight(-6.0)) {
			s.climbingState();
		} else if (coDriver.POV90.shortReleased() && !jacks.isAtHeight(-6.0)) {
			s.shortClimbingState();
		} else if (coDriver.POV180.shortReleased()) {
			s.postClimbingState();
		} else if (coDriver.POV270.shortReleased()) {
			//s.lockedJackState();
			if(jacks.isAtHeight(-6.0)){
				s.jackState(Constants.kJackMaxControlHeight);
			}else{
				s.jackState(-6.0);
				wrist.setAngle(Constants.kWristMaxControlAngle);
			}
		}

		if (diskScorer.needsToNotifyDrivers() || ballCarriage.needsToNotifyDrivers() || swerve.needsToNotifyDrivers()) {
			driver.rumble(1.0, 2.0);
			coDriver.rumble(1.0, 2.0);
		}

		if(coDriver.leftBumper.longPressed()){ 
			elevator.setManualSpeed(0.5);
			elevator.enableLimits(false); 
			coDriver.rumble(1.0, 1.0); 
		}else if(!elevator.limitsEnabled() && coDriver.leftBumper.longReleased()){
			elevator.zeroSensors(); 
			elevator.enableLimits(true);
			elevator.setManualSpeed(Constants.kElevatorTeleopManualSpeed);
			elevator.lockHeight(); 
        } 
        
        if(subsystems.haveEmergency()){
            leds.conformToState(LEDs.State.EMERGENCY);
        } else if (s.isClimbing()) {
            leds.conformToState(LEDs.State.CLIMBING);
        } else if (swerve.getState() == Swerve.ControlState.VISION_TRAJECTORY){
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
    }

    private void oneControllerMode() {

    }
}

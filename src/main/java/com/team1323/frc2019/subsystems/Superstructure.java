package com.team1323.frc2019.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2019.Constants;
import com.team1323.frc2019.RobotState;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.LimelightProcessor;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.Swerve.VisionState;
import com.team1323.frc2019.subsystems.requests.LambdaRequest;
import com.team1323.frc2019.subsystems.requests.ParallelRequest;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team1323.frc2019.subsystems.requests.SequentialRequest;
import com.team1323.lib.util.InterpolatingDouble;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {

	public Elevator elevator;
	public Wrist wrist;
	public BallIntake ballIntake;
	public BallCarriage ballCarriage;
	public DiskIntake diskIntake;
	public DiskScorer diskScorer;
	public Jacks jacks;

	private Compressor compressor;
	
	public Swerve swerve;

	private LimelightProcessor limelight;

	private RobotState robotState;

	private boolean isClimbing = false;
	public boolean isClimbing(){ return isClimbing; }
	public void enableInterpolator(boolean enable){ 
		isClimbing = enable;
		swerve.setLowPowerScalar(0.25);
	}
	public void stopClimbing(){ 
		isClimbing = false; 
		swerve.setLowPowerScalar(0.6);
	}
	
	public Superstructure(){
		elevator = Elevator.getInstance();
		wrist = Wrist.getInstance();
		ballIntake = BallIntake.getInstance();
		ballCarriage = BallCarriage.getInstance();
		diskIntake = DiskIntake.getInstance();
		diskScorer = DiskScorer.getInstance();
		jacks = Jacks.getInstance();
		
		compressor = new Compressor(20);
		
		swerve = Swerve.getInstance();

		limelight = LimelightProcessor.getInstance();

		robotState = RobotState.getInstance();
		
		queuedRequests = new ArrayList<>(0);
	}
	private static Superstructure instance = null;
	public static Superstructure getInstance(){
		if(instance == null)
			instance = new Superstructure();
		return instance;
	}

	private Request activeRequest = null;
	private List<Request> queuedRequests = new ArrayList<>();
	
	private boolean newRequest = false;
	private boolean allRequestsCompleted = false;
	public boolean requestsCompleted(){ return allRequestsCompleted; }
	
	private void setActiveRequest(Request request){
		activeRequest = request;
		newRequest = true;
		allRequestsCompleted = false;
	}
	
	private void setQueue(List<Request> requests){
		clearQueue();
		for(Request request : requests) {
			queuedRequests.add(request);
		}
	}

	private void setQueue(Request request) {
		setQueue(Arrays.asList(request));
	}

	private void clearQueue() {
		queuedRequests.clear();
	}
	
	public void request(Request r){
		setActiveRequest(r);
		clearQueue();
	}
	
	public void request(Request active, Request queue){
		setActiveRequest(active);
		setQueue(queue);
	}
	
	public void queue(Request request){
		queuedRequests.add(request);
	}
	
	public void replaceQueue(Request request){
		setQueue(request);
	}
	
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			stop();
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Superstructure.this){

				boolean inRange = elevator.inVisionRange(diskScorer.hasDisk() ? Constants.kElevatorDiskVisibleRanges : Constants.kElevatorBallVisibleRanges);
				limelight.enableUpdates(inRange);
				if(!inRange)
					robotState.clearVisionTargets();
			
				double elevatorHeight = elevator.getHeight();
				swerve.setMaxSpeed(Constants.kSwerveSpeedTreeMap.getInterpolated(new InterpolatingDouble(elevatorHeight)).value);

				if(isClimbing())
					jacks.setHeight(Constants.kJackHeightTreeMap.getInterpolated(new InterpolatingDouble(wrist.getAngle())).value);
					//wrist.setAngle(Constants.kWristAngleTreemap.getInterpolated(new InterpolatingDouble(jacks.getHeight())).value);

				if(newRequest) {
					activeRequest.act();
					newRequest = false;
				} 

				if(activeRequest == null) {
					if(queuedRequests.isEmpty()) {
						allRequestsCompleted = true;
					} else {
						setActiveRequest(queuedRequests.remove(0));
					}
				} else if(activeRequest.isFinished()) {
					activeRequest = null;
				}
			
			}
		}

		@Override
		public void onStop(double timestamp) {
			disabledState();
		}
		
	};
	
	public synchronized void sendManualInput(double wristOutput, double elevatorOutput, double jackOutput){
		List<Request> list = new ArrayList<>();
		if(wristOutput != 0){
			list.add(wrist.openLoopRequest(wristOutput));
		}else if(wrist.isOpenLoop()){
			list.add(wrist.lockAngleRequest());
		}
		if(elevatorOutput != 0){
			list.add(elevator.openLoopRequest(elevatorOutput));
		}else if(elevator.isOpenLoop()){
			list.add(elevator.lockHeightRequest());
		}
		/*if(jackOutput != 0){
			list.add(jacks.openLoopRequest(jackOutput));
		}else if(jacks.isOpenLoop()){
			list.add(jacks.lockHeightRequest());
		}*/
		
		if(!list.isEmpty()){
			request(new ParallelRequest(list));
		}
	}

	public void sendJackInput(double input){
		List<Request> list = new ArrayList<>();
		/*if(input != 0){
			list.add(jacks.openLoopRequest(input));
		}else if(jacks.isOpenLoop()){
			list.add(jacks.lockHeightRequest());
		}*/
		//list.add(jacks.openLoopRequest(input));

		if(!list.isEmpty()){
			request(new ParallelRequest(list));
		}
	}
	
	public void enableCompressor(boolean enable){
		compressor.setClosedLoopControl(enable);
	}

	@Override
	public void stop() {
	}

	@Override
	public void zeroSensors() {
		
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putBoolean("Superstructure Done", allRequestsCompleted);
		SmartDashboard.putString("Active Request", (activeRequest == null) ? "null" : activeRequest.toString());
	}

	public Request waitRequest(double seconds){
		return new Request(){
			double startTime = 0.0;
			double waitTime = 1.0;
		
			@Override
			public void act() {
				startTime = Timer.getFPGATimestamp();
				waitTime = seconds;
			}

			@Override
			public boolean isFinished(){
				return (Timer.getFPGATimestamp() - startTime) > waitTime;
			}
		};
	}

	public Request waitForVisionRequest(){
		return new Request(){

			@Override
			public void act() {

			}

			@Override
			public boolean isFinished(){
				return robotState.seesTarget();
			}

		};
	}

	/////States/////

	public void disabledState(){
		request(new ParallelRequest(
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			wrist.angleRequest(Constants.kWristPrimaryStowAngle),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED)
		));
	}

	public void neutralState(){
		request(new ParallelRequest(
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(diskScorer.isExtended() ? DiskScorer.State.NEUTRAL_EXTENDED : DiskScorer.State.STOWED)
		));
	}

	public void ballIntakingState(){
		request(new ParallelRequest(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristIntakingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.INTAKING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			ballIntake.waitForBallRequest()
		));
	}

	public void ballHoldingState(){
		request(new ParallelRequest(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristBallHoldingAngle),
			ballCarriage.stateRequest(BallCarriage.State.RECEIVING), 
			ballIntake.stateRequest(BallIntake.State.HOLDING),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF)
		)); 
	}

	public void ballFeedingState(){
		request(new SequentialRequest(
			new ParallelRequest(
				elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
				wrist.angleRequest(Constants.kWristBallFeedingAngle, 0.1),
				ballCarriage.stateRequest(BallCarriage.State.RECEIVING), 
				ballIntake.stateRequest(BallIntake.State.FEEDING),
				diskScorer.stateRequest(DiskScorer.State.STOWED),
				diskIntake.stateRequest(DiskIntake.State.OFF),
				ballCarriage.waitForBallRequest()
			),
			new ParallelRequest(
				ballIntake.stateRequest(BallIntake.State.POST_FEEDING),
				ballCarriage.stateRequest(BallCarriage.State.OFF)
			)
		));
	}

	public void fullBallCycleState(){
		request(new SequentialRequest(
			new ParallelRequest(
				elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
				wrist.angleRequest(Constants.kWristBallHoldingAngle),
				ballCarriage.stateRequest(BallCarriage.State.RECEIVING),
				ballIntake.stateRequest(BallIntake.State.HOLDING),
				diskScorer.stateRequest(DiskScorer.State.STOWED),
				diskIntake.stateRequest(DiskIntake.State.OFF)
			),
			new ParallelRequest(
				elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
				wrist.angleRequest(Constants.kWristBallFeedingAngle, 0.5, false), 
				ballIntake.stateRequest(BallIntake.State.FEEDING),
				diskIntake.stateRequest(DiskIntake.State.OFF),
				ballCarriage.waitForBallRequest()
			),
			new ParallelRequest(
				ballIntake.stateRequest(BallIntake.State.POST_FEEDING),
				elevator.heightRequest(Constants.kElevatorLowBallHeight)
			)
		));
	}

	public void ballScoringState(double elevatorHeight){
		request(new ParallelRequest(
			elevator.heightRequest(elevatorHeight), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF)
		)); 
	}

	public void ballTrackingState(double elevatorHeight){
		request(new SequentialRequest(
			elevator.heightRequest(elevator.nearestVisionHeight(Constants.kElevatorBallVisibleRanges)),
			swerve.startTrackRequest(Constants.kBallTargetHeight, new Translation2d(-1.0, 0.0), false, VisionState.LINEAR),
			waitRequest(0.5),
			elevator.heightRequest(elevator.nearestVisionHeight(elevatorHeight, Constants.kElevatorBallVisibleRanges)), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest(),
			elevator.heightRequest(elevatorHeight),
			ballCarriage.stateRequest(BallCarriage.State.EJECTING)
		)); 
	}

	public void diskIntakingState(){
		request(new SequentialRequest(
			new ParallelRequest(
				elevator.heightRequest(Constants.kElevatorDiskIntakeHeight),
				diskScorer.stateRequest(DiskScorer.State.STOWED),
				diskIntake.stateRequest(DiskIntake.State.INTAKING),
				ballIntake.stateRequest(BallIntake.State.OFF),
				ballCarriage.stateRequest(BallCarriage.State.OFF),
				wrist.angleRequest(Constants.kWristPrimaryStowAngle),
				diskIntake.waitForDiskRequest()
			),
			diskIntake.stateRequest(DiskIntake.State.DELIVERING),
			diskScorer.stateRequest(DiskScorer.State.GROUND_INTAKING),
			diskScorer.waitForDiskRequest(),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.GROUND_DETECTED),
			elevator.heightRequest(Constants.kElevatorDiskHandoffHeight),
			diskIntake.stateRequest(DiskIntake.State.HANDOFF_COMPLETE),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			waitRequest(0.5),
			elevator.heightRequest(Constants.kElevatorLowHatchHeight)
		)); 
	}

	public void diskReceivingState(){
		request(new SequentialRequest(
			new ParallelRequest(
				elevator.heightRequest(Constants.kElevatorHumanLoaderHeight),
				diskScorer.stateRequest(DiskScorer.State.AUTO_RECEIVING),
				wrist.angleRequest(Constants.kWristPrimaryStowAngle),
				diskIntake.stateRequest(DiskIntake.State.OFF),
				ballIntake.stateRequest(BallIntake.State.OFF),
				ballCarriage.stateRequest(BallCarriage.State.OFF),
				diskScorer.waitForDiskRequest()
			),
			new ParallelRequest(
				diskScorer.stateRequest(DiskScorer.State.DETECTED),
				diskIntake.stateRequest(DiskIntake.State.HANDOFF_COMPLETE)
			)
		));
	}

	public void diskScoringState(){
		request(new SequentialRequest(
			diskScorer.ejectRequest(elevator.getHeight()),
			swerve.trajectoryRequest(new Translation2d(-24.0, 0.0), swerve.getPose().getRotation().getUnboundedDegrees(), 36.0)
		)); 
	}

	public void diskScoringState(double elevatorHeight, boolean resuck){
		request(new ParallelRequest(
			elevator.heightRequest(elevatorHeight),
			wrist.angleRequest(Constants.kWristPrimaryStowAngle),
			diskScorer.stateRequest(resuck ? DiskScorer.State.DETECTED : DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF)
		)); 
	}

	/** Used for driver tracking */
	public void diskTrackingState(){
		request(new SequentialRequest(
			waitForVisionRequest(),
			swerve.startTrackRequest(Constants.kDiskTargetHeight, new Translation2d(-1.0, 0.0), true, VisionState.LINEAR),
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest(),
			swerve.strictWaitForTrackRequest()
		)); 
	}

	/** Used for codriver tracking */
	public void diskTrackingState(double elevatorHeight){
		request(new SequentialRequest(
			elevator.heightRequest(elevator.nearestVisionHeight(Constants.kElevatorDiskVisibleRanges)),
			waitForVisionRequest(),
			swerve.startTrackRequest(Constants.kDiskTargetHeight, new Translation2d(-1.0, 0.0), true, VisionState.LINEAR),
			waitRequest(0.25),
			elevator.heightRequest(elevator.nearestVisionHeight(elevatorHeight, Constants.kElevatorDiskVisibleRanges)), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest(),
			elevator.heightRequest(elevatorHeight),
			swerve.strictWaitForTrackRequest()
		)); 
	}

	public void diskTrackingState(double elevatorHeight, Rotation2d fixedOrientation){
		request(new SequentialRequest(
			waitForVisionRequest(),
			swerve.startTrackRequest(Constants.kDiskTargetHeight, new Translation2d(-5.0, 0.0), true, fixedOrientation),
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest(),
			elevator.heightRequest(elevatorHeight),
			diskScorer.stateRequest(DiskScorer.State.SCORING)
		)); 
	}

	public void diskTrackingState(double elevatorHeight, Rotation2d fixedOrientation, double trackingSpeed){
		request(new SequentialRequest(
			waitForVisionRequest(),
			swerve.startTrackRequest(Constants.kDiskTargetHeight, new Translation2d(-5.0, 0.0), true, fixedOrientation, Constants.kClosestVisionDistance, trackingSpeed),
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest(),
			elevator.heightRequest(elevatorHeight),
			diskScorer.stateRequest(DiskScorer.State.SCORING)
		)); 
	}

	public void diskTrackingState(double elevatorHeight, Rotation2d fixedOrientation, double cutoffDistance, Translation2d endTranslation, double trackingSpeed){
		request(new SequentialRequest(
			waitForVisionRequest(),
			swerve.startTrackRequest(Constants.kDiskTargetHeight, endTranslation, true, fixedOrientation, cutoffDistance, trackingSpeed),
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest(),
			elevator.heightRequest(elevatorHeight),
			diskScorer.stateRequest(DiskScorer.State.SCORING)
		)); 
	}

	public void humanLoaderTrackingState(){
		request(new SequentialRequest(
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			elevator.heightRequest(elevator.nearestVisionHeight(Constants.kElevatorBallVisibleRanges)),
			waitForVisionRequest(),
			swerve.trackRequest(Constants.kDiskTargetHeight, new Translation2d(/*7.0*/4.0, 0.0), false, Rotation2d.fromDegrees(180.0), 48.0, 54.0),
			new ParallelRequest(
				elevator.heightRequest(Constants.kElevatorHumanLoaderHeight),
				diskScorer.stateRequest(DiskScorer.State.AUTO_RECEIVING),
				diskScorer.waitForDiskRequest()
			),
			diskScorer.stateRequest(DiskScorer.State.DETECTED)
		));
	}

	public void humanLoaderRetrievingState(){
		robotState.clearVisionTargets(); //TODO test this addition to the sequence
		request(new SequentialRequest(
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.NEUTRAL_EXTENDED),
			elevator.heightRequest(elevator.nearestVisionHeight(Constants.kElevatorBallVisibleRanges)),
			waitForVisionRequest(),
			swerve.trackRequest(Constants.kDiskTargetHeight, new Translation2d(4.0, 0.0), false, Rotation2d.fromDegrees(180.0), 66.0, 60.0),
			new ParallelRequest(
				elevator.heightRequest(Constants.kElevatorHumanLoaderHeight),
				diskScorer.stateRequest(DiskScorer.State.RECEIVING),
				diskScorer.waitForDiskRequest()
			),
			diskScorer.stateRequest(DiskScorer.State.DETECTED),
			swerve.trajectoryRequest(new Translation2d(-60.0, 0.0), 180.0, 60.0),
			swerve.openLoopRequest(new Translation2d(), 0.0)
		));
	}

	public void shortClimbingState(){
		request(new SequentialRequest(
			wrist.angleRequest(Constants.kWristShortPlatformAngle),
			new ParallelRequest(
				wrist.gearShiftRequest(false),
				jacks.shiftPowerRequest(true),
				elevator.heightRequest(Constants.kElevatorLowBallHeight),
				diskScorer.stateRequest(DiskScorer.State.STOWED),
				ballIntake.stateRequest(BallIntake.State.CLIMBING),
				ballCarriage.stateRequest(BallCarriage.State.OFF),
				jacks.heightRequest(Constants.kJackShortClimbHeight),
				wrist.angleRequest(Constants.kWristShortHangingAngle)
			),
			new ParallelRequest(
				ballIntake.stateRequest(BallIntake.State.PULLING),
				swerve.velocityRequest(Rotation2d.fromDegrees(180.0), 48.0)
			)
		));
	}

	public void climbingState(){
		request(new SequentialRequest(
			wrist.angleRequest(37.0),
			new ParallelRequest(
				wrist.gearShiftRequest(false),
				jacks.shiftPowerRequest(true),
				new LambdaRequest(() -> enableInterpolator(true)),
				elevator.heightRequest(Constants.kElevatorLowBallHeight),
				diskScorer.stateRequest(DiskScorer.State.STOWED),
				ballIntake.stateRequest(BallIntake.State.CLIMBING),
				ballCarriage.stateRequest(BallCarriage.State.OFF),
				wrist.angleRequest(Constants.kWristHangingAngle)
			),
			new ParallelRequest(
				ballIntake.stateRequest(BallIntake.State.PULLING),
				swerve.velocityRequest(Rotation2d.fromDegrees(180.0), 48.0)
			)
		));
	}

	public void postClimbingState(){
		isClimbing = false;
		request(new SequentialRequest(
			new ParallelRequest(
				wrist.gearShiftRequest(false),
				elevator.heightRequest(Constants.kElevatorLowBallHeight),
				diskScorer.stateRequest(DiskScorer.State.STOWED),
				ballIntake.stateRequest(BallIntake.State.OFF),
				ballCarriage.stateRequest(BallCarriage.State.OFF),
				swerve.velocityRequest(Rotation2d.fromDegrees(180.0), 18.0),
				wrist.angleRequest(Constants.kWristBallFeedingAngle),
				jacks.heightRequest(Constants.kJackStartingHeight)
			),
			new ParallelRequest(
				jacks.shiftPowerRequest(false),
				wrist.gearShiftRequest(true)
			)
		));
	}

	public void lockedJackState(){
		request(new SequentialRequest(
			jacks.shiftPowerRequest(true),
			jacks.heightRequest(Constants.kJackMaxControlHeight)
		));
	}

	public void jackState(double jackHeight){
		request(new SequentialRequest(
			jacks.shiftPowerRequest(true),
			jacks.heightRequest(jackHeight)
		));
	}

}

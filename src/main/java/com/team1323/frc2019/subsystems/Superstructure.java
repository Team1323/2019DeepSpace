package com.team1323.frc2019.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import com.team1323.frc2019.Constants;
import com.team1323.frc2019.RobotState;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.LimelightProcessor;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team1323.frc2019.subsystems.requests.RequestList;
import com.team1323.lib.util.InterpolatingDouble;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends Subsystem {

	public Elevator elevator;
	public Wrist wrist;
	public BallIntake ballIntake;
	public BallCarriage ballCarriage;
	public DiskIntake diskIntake;
	public DiskScorer diskScorer;
	//public Jacks jacks;

	private Compressor compressor;
	
	private Swerve swerve;

	private LimelightProcessor limelight;

	private RobotState robotState;

	private boolean isClimbing = false;
	public boolean isClimbing(){ return isClimbing; }
	public void startClimbing(){ isClimbing = true; }
	public void stopClimbing(){ isClimbing = false; }
	
	public Superstructure(){
		elevator = Elevator.getInstance();
		wrist = Wrist.getInstance();
		ballIntake = BallIntake.getInstance();
		ballCarriage = BallCarriage.getInstance();
		diskIntake = DiskIntake.getInstance();
		diskScorer = DiskScorer.getInstance();
		//jacks = Jacks.getInstance();
		
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
	
	private RequestList activeRequests;
	private ArrayList<RequestList> queuedRequests;
	private Request currentRequest;
	
	private boolean newRequests = false;
	private boolean activeRequestsCompleted = false;
	private boolean allRequestsCompleted = false;
	public boolean requestsCompleted(){ return allRequestsCompleted; }
	
	private void setActiveRequests(RequestList requests){
		activeRequests = requests;
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}
	
	private void setQueuedRequests(RequestList requests){
		queuedRequests.clear();
		queuedRequests.add(requests);
	}
	
	private void setQueuedRequests(List<RequestList> requests){
		queuedRequests.clear();
		queuedRequests = new ArrayList<>(requests.size());
		for(RequestList list : requests){
			queuedRequests.add(list);
		}
	}
	
	public void request(Request r){
		setActiveRequests(new RequestList(Arrays.asList(r), false));
		setQueuedRequests(new RequestList());
	}
	
	public void request(Request active, Request queue){
		setActiveRequests(new RequestList(Arrays.asList(active), false));
		setQueuedRequests(new RequestList(Arrays.asList(queue), false));
	}
	
	public void request(RequestList list){
		setActiveRequests(list);
		setQueuedRequests(new RequestList());
	}
	
	public void request(RequestList activeList, RequestList queuedList){
		setActiveRequests(activeList);
		setQueuedRequests(queuedList);
	}
	
	public void addActiveRequest(Request request){
		activeRequests.add(request);
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}
	
	/** Ill-advised */
	public void addForemostActiveRequest(Request request){
		activeRequests.addToForefront(request);
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}
	
	public void queue(Request request){
		queuedRequests.add(new RequestList(Arrays.asList(request), false));
	}
	
	public void queue(RequestList list){
		queuedRequests.add(list);
	}
	
	public void replaceQueue(Request request){
		setQueuedRequests(new RequestList(Arrays.asList(request), false));
	}
	
	public void replaceQueue(RequestList list){
		setQueuedRequests(list);
	}
	
	public void replaceQueue(List<RequestList> lists){
		setQueuedRequests(lists);
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

				//if(isClimbing())
				//	jacks.setHeight(Constants.kJackHeightTreeMap.getInterpolated(new InterpolatingDouble(wrist.getAngle())).value);
				
				if(!activeRequestsCompleted){
					if(newRequests){
						if(activeRequests.isParallel()){
							boolean allActivated = true;
							for(Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator.hasNext();){
								Request request = iterator.next();
								boolean allowed = request.allowed();
								allActivated &= allowed;
								if(allowed) request.act();
							}
							newRequests = !allActivated;
						}else{
							if(activeRequests.isEmpty()){
								activeRequestsCompleted = true;
								return;
							}
							currentRequest = activeRequests.remove();
							currentRequest.act();
							newRequests = false;
						}
					}
					if(activeRequests.isParallel()){
						boolean done = true;
						for(Request request : activeRequests.getRequests()){
							done &= request.isFinished();
						}
						activeRequestsCompleted = done;
					}else if(currentRequest.isFinished()){
							if(activeRequests.isEmpty()){
								activeRequestsCompleted = true;
							}else if(activeRequests.getRequests().get(0).allowed()){
								newRequests = true;
								activeRequestsCompleted = false;
							}
					}
				}else{
					if(!queuedRequests.isEmpty()){
						setActiveRequests(queuedRequests.remove(0));
					}else{
						allRequestsCompleted = true;
					}
				}
			
			}
		}

		@Override
		public void onStop(double timestamp) {
			disabledState();
		}
		
	};
	
	public void sendManualInput(double wristOutput, double elevatorOutput, double jackOutput){
		RequestList list = RequestList.emptyList();
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
			request(list);
		}
	}

	public void sendJackInput(double input){
		RequestList list = RequestList.emptyList();
		/*if(input != 0){
			list.add(jacks.openLoopRequest(input));
		}else if(jacks.isOpenLoop()){
			list.add(jacks.lockHeightRequest());
		}*/
		//list.add(jacks.openLoopRequest(input));

		if(!list.isEmpty()){
			request(list);
		}
	}
	
	public void enableCompressor(boolean enable){
		compressor.setClosedLoopControl(enable);
	}
	
	public RequestList elevatorWristConfig(double elevatorHeight, double wristAngle){
		return new RequestList(Arrays.asList(elevator.heightRequest(elevatorHeight),
				wrist.angleRequest(wristAngle)), true);
	}
	
	public RequestList idleRequest(){
		return new RequestList(Arrays.asList(wrist.openLoopRequest(0.0),
				elevator.openLoopRequest(0.0)), true);
	}

	@Override
	public void stop() {
		setActiveRequests(idleRequest());
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

	/////States/////

	public void disabledState(){
		RequestList state = new RequestList(Arrays.asList(
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			wrist.angleRequest(Constants.kWristPrimaryStowAngle),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED)), true);
		request(state);
	}

	public void neutralState(){
		RequestList state = new RequestList(Arrays.asList(
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			wrist.angleRequest(Constants.kWristPrimaryStowAngle),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(diskScorer.isExtended() ? DiskScorer.State.NEUTRAL_EXTENDED : DiskScorer.State.STOWED)), true);
		request(state);
	}

	public void ballIntakingState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristIntakingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.INTAKING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			ballIntake.waitForBallRequest()), true);
		/*RequestList queue = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristBallHoldingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.HOLDING)), true);*/
		request(state);
	}

	public void ballHoldingState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristBallHoldingAngle),
			ballCarriage.stateRequest(BallCarriage.State.RECEIVING), 
			ballIntake.stateRequest(BallIntake.State.HOLDING),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF)), true);
		request(state); 
	}

	public void ballFeedingState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle, 0.1),
			ballCarriage.stateRequest(BallCarriage.State.RECEIVING), 
			ballIntake.stateRequest(BallIntake.State.FEEDING),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballCarriage.waitForBallRequest()), true);
		RequestList queue = new RequestList(Arrays.asList(
			ballIntake.stateRequest(BallIntake.State.POST_FEEDING),
			ballCarriage.stateRequest(BallCarriage.State.OFF)), true);
		request(state, queue);
	}

	public void fullBallCycleState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristBallHoldingAngle),
			ballCarriage.stateRequest(BallCarriage.State.RECEIVING),
			ballIntake.stateRequest(BallIntake.State.HOLDING),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF)), true);
		List<RequestList> queue = Arrays.asList(
			new RequestList(Arrays.asList(
				elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
				wrist.angleRequest(Constants.kWristBallFeedingAngle, 0.1, false), 
				ballIntake.stateRequest(BallIntake.State.FEEDING),
				diskIntake.stateRequest(DiskIntake.State.OFF),
				ballCarriage.waitForBallRequest()), true),
			new RequestList(Arrays.asList(
				ballIntake.stateRequest(BallIntake.State.POST_FEEDING),
				elevator.heightRequest(Constants.kElevatorLowBallHeight)), true)
		);
		request(state);
		replaceQueue(queue);
	}

	public void ballScoringState(double elevatorHeight){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(elevatorHeight), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF)), true);
		request(state); 
	}

	public void ballTrackingState(double elevatorHeight){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(elevator.nearestVisionHeight(Constants.kElevatorBallVisibleRanges)),
			swerve.startTrackRequest(Constants.kBallTargetHeight, Constants.kRobotProbeExtrusion, false),
			waitRequest(0.5),
			elevator.heightRequest(elevator.nearestVisionHeight(elevatorHeight, Constants.kElevatorBallVisibleRanges)), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest()), false);
		RequestList queue = new RequestList(Arrays.asList(
			elevator.heightRequest(elevatorHeight),
			ballCarriage.stateRequest(BallCarriage.State.EJECTING)), false);
		request(state, queue); 
	}

	public void diskIntakingState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorDiskIntakeHeight),
			diskScorer.stateRequest(DiskScorer.State.GROUND_INTAKING),
			diskIntake.stateRequest(DiskIntake.State.INTAKING),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			wrist.angleRequest(Constants.kWristPrimaryStowAngle),
			diskIntake.waitForDiskRequest()), true);
		RequestList queue = new RequestList(Arrays.asList(
			diskIntake.stateRequest(DiskIntake.State.OFF),
			diskScorer.waitForDiskRequest(),
			waitRequest(0.75),
			diskScorer.stateRequest(DiskScorer.State.NEUTRAL_EXTENDED),
			waitRequest(0.5),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			elevator.heightRequest(Constants.kElevatorLowHatchHeight),
			diskIntake.stateRequest(DiskIntake.State.HANDOFF_COMPLETE)), false);
		request(state, queue); 
	}

	public void diskReceivingState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorHumanLoaderHeight),
			diskScorer.stateRequest(DiskScorer.State.AUTO_RECEIVING),
			wrist.angleRequest(Constants.kWristPrimaryStowAngle),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			diskScorer.waitForDiskRequest()), true);
		RequestList queue = new RequestList(Arrays.asList(
			diskScorer.stateRequest(DiskScorer.State.DETECTED),
			diskIntake.stateRequest(DiskIntake.State.HANDOFF_COMPLETE)), true);
		request(state, queue); 
	}

	public void diskScoringState(double elevatorHeight){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(elevatorHeight),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF)), true);
		request(state); 
	}

	public void diskTrackingState(double elevatorHeight){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(elevator.nearestVisionHeight(Constants.kElevatorDiskVisibleRanges)),
			swerve.startTrackRequest(Constants.kHatchTargetHeight, Constants.kRobotProbeExtrusion, true),
			waitRequest(0.5),
			elevator.heightRequest(elevator.nearestVisionHeight(elevatorHeight, Constants.kElevatorDiskVisibleRanges)), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest()), false);
		RequestList queue = new RequestList(Arrays.asList(
			elevator.heightRequest(elevatorHeight),
			diskScorer.stateRequest(DiskScorer.State.SCORING)), false);
		request(state, queue); 
	}

	public void diskTrackingState(double elevatorHeight, Rotation2d fixedOrientation){
		RequestList state = new RequestList(Arrays.asList(
			//elevator.heightRequest(elevator.nearestVisionHeight(Constants.kElevatorDiskVisibleRanges)),
			swerve.startTrackRequest(Constants.kHatchTargetHeight, 3.0, true, fixedOrientation),
			waitRequest(0.5),
			//elevator.heightRequest(elevator.nearestVisionHeight(elevatorHeight, Constants.kElevatorDiskVisibleRanges)), 
			wrist.angleRequest(Constants.kWristBallFeedingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			swerve.waitForTrackRequest()), false);
		RequestList queue = new RequestList(Arrays.asList(
			waitRequest(0.25),
			elevator.heightRequest(elevatorHeight),
			diskScorer.stateRequest(DiskScorer.State.SCORING)), false);
		request(state, queue); 
	}

	public void humanLoaderTrackingState(){
		RequestList state = new RequestList(Arrays.asList(
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.HOLDING),
			elevator.heightRequest(elevator.nearestVisionHeight(Constants.kElevatorBallVisibleRanges)),
			swerve.trackRequest(Constants.kHatchTargetHeight, -7.0, false, Rotation2d.fromDegrees(180.0))), false);

		List<RequestList> queue = Arrays.asList(
			new RequestList(Arrays.asList(
				elevator.heightRequest(Constants.kElevatorHumanLoaderHeight),
				diskScorer.stateRequest(DiskScorer.State.AUTO_RECEIVING),
				diskScorer.waitForDiskRequest()), true),
			new RequestList(Arrays.asList(
				//swerve.waitForTrackRequest(),
				diskScorer.stateRequest(DiskScorer.State.DETECTED)), false)
		);
		request(state); 
		replaceQueue(queue);
	}

	public void humanLoaderRetrievingState(){
		RequestList state = new RequestList(Arrays.asList(
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			diskScorer.stateRequest(DiskScorer.State.NEUTRAL_EXTENDED),
			elevator.heightRequest(elevator.nearestVisionHeight(Constants.kElevatorBallVisibleRanges)),
			//waitRequest(0.5),
			swerve.trackRequest(Constants.kHatchTargetHeight, -7.0, false, Rotation2d.fromDegrees(180.0))), false);

		List<RequestList> queue = Arrays.asList(
			new RequestList(Arrays.asList(
				elevator.heightRequest(Constants.kElevatorHumanLoaderHeight),
				diskScorer.stateRequest(DiskScorer.State.RECEIVING),
				diskScorer.waitForDiskRequest()), true),
			new RequestList(Arrays.asList(
				swerve.waitForTrackRequest(),
				diskScorer.stateRequest(DiskScorer.State.DETECTED),
				swerve.trajectoryRequest(new Translation2d(-84.0, 0.0), 0.0),
				swerve.openLoopRequest(new Translation2d(), 0.0)), false)
		);
		request(state); 
		replaceQueue(queue);
	}

	public void climbingState(){
		RequestList state = new RequestList(Arrays.asList(
			wrist.gearShiftRequest(false),
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.CLIMBING),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			wrist.angleRequest(Constants.kWristHangingAngle)), true);
		request(state);
		isClimbing = true;
	}

	public void postClimbingState(){
		isClimbing = false;
		RequestList state = new RequestList(Arrays.asList(
			wrist.gearShiftRequest(false),
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight),
			diskScorer.stateRequest(DiskScorer.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.CLIMBING),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			wrist.angleRequest(Constants.kWristBallFeedingAngle)/*,
			jacks.heightRequest(Constants.kJackStartingHeight)*/), true);
		request(state);
	}

}

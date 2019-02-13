package com.team1323.frc2019.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import com.team1323.frc2019.Constants;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team1323.frc2019.subsystems.requests.RequestList;
import com.team1323.lib.util.InterpolatingDouble;

import edu.wpi.first.wpilibj.Compressor;

public class Superstructure extends Subsystem {

	public Elevator elevator;
	public Wrist wrist;
	public BallIntake ballIntake;
	public BallCarriage ballCarriage;
	public DiskIntake diskIntake;
	public Probe probe;
	public Jacks jacks;
	
	private Compressor compressor;
	
	private Swerve swerve;

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
		probe = Probe.getInstance();
		jacks = Jacks.getInstance();
		
		compressor = new Compressor(20);
		
		swerve = Swerve.getInstance();
		
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
			
				double elevatorHeight = elevator.getHeight();
				swerve.setMaxSpeed(Constants.kSwerveSpeedTreeMap.getInterpolated(new InterpolatingDouble(elevatorHeight)).value);

				if(isClimbing())
					jacks.setHeight(Constants.kJackHeightTreeMap.getInterpolated(new InterpolatingDouble(wrist.getAngle())).value);
				
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
		if(jackOutput != 0){
			list.add(jacks.openLoopRequest(jackOutput));
		}else if(jacks.isOpenLoop()){
			list.add(jacks.lockHeightRequest());
		}
		
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
		list.add(jacks.openLoopRequest(input));

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

	/////States/////

	public RequestList ballIntakingState(){
		return new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorBallIntakeHeight), 
			wrist.angleRequest(Constants.kWristIntakingAngle),
			ballCarriage.stateRequest(BallCarriage.State.OFF), 
			ballIntake.stateRequest(BallIntake.State.INTAKING),
			diskIntake.stateRequest(DiskIntake.State.OFF)), true);
	}

	public RequestList diskIntakingState(){
		return new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorDiskIntakeHeight),
			probe.stateRequest(Probe.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.INTAKING),
			ballIntake.stateRequest(BallIntake.State.OFF),
			ballCarriage.stateRequest(BallCarriage.State.OFF)), true);
	}

	public RequestList climbingState(){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(Constants.kElevatorDiskIntakeHeight),
			probe.stateRequest(Probe.State.STOWED),
			diskIntake.stateRequest(DiskIntake.State.OFF),
			ballIntake.stateRequest(BallIntake.State.HOLDING),
			ballCarriage.stateRequest(BallCarriage.State.OFF),
			wrist.angleRequest(Constants.kWristHangingAngle)), true);
		request(state);
		isClimbing = true;
		return state;
	}

}

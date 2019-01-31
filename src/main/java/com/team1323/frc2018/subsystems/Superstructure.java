package com.team1323.frc2018.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.loops.ILooper;
import com.team1323.frc2018.loops.Loop;
import com.team1323.frc2018.subsystems.Intake.IntakeState;
import com.team1323.frc2018.subsystems.requests.Request;
import com.team1323.frc2018.subsystems.requests.RequestList;
import com.team1323.lib.util.InterpolatingDouble;

import edu.wpi.first.wpilibj.Compressor;

public class Superstructure extends Subsystem {

	public Intake intake;
	public Wrist wrist;
	public Elevator elevator;
	
	private Compressor compressor;
	
	private Swerve swerve;
	
	public Superstructure(){
		intake = Intake.getInstance();
		wrist = Wrist.getInstance();
		elevator = Elevator.getInstance();
		
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
	
	public void sendManualInput(double wristOutput, double elevatorOutput){
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
	
	public RequestList wristIntakeConfig(double wristAngle, IntakeState intakeState){
		return new RequestList(Arrays.asList(wrist.angleRequest(wristAngle), intake.stateRequest(intakeState)), true);
	}
	
	public RequestList elevatorWristIntakeConfig(double elevatorHeight, double wristAngle, IntakeState intakeState){
		return new RequestList(Arrays.asList(elevator.heightRequest(elevatorHeight),
				wrist.angleRequest(wristAngle), intake.stateRequest(intakeState)), true);
	}
	
	public RequestList idleRequest(){
		return new RequestList(Arrays.asList(wrist.openLoopRequest(0.0),
				elevator.openLoopRequest(0.0), intake.stateRequest(IntakeState.OFF)), true);
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

}

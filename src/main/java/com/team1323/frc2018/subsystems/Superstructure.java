package com.team1323.frc2018.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1323.frc2018.Constants;
import com.team1323.frc2018.Ports;
import com.team1323.frc2018.loops.ILooper;
import com.team1323.frc2018.loops.Loop;
import com.team1323.frc2018.subsystems.Intake.IntakeState;
import com.team1323.lib.util.InterpolatingDouble;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem{
	
	public Intake intake;
	public Wrist wrist;
	public Elevator elevator;
	
	private TalonSRX winch;
	private boolean winchSetpointSet = false;
	
	private Compressor compressor;
	
	private Swerve swerve;
	
	public Superstructure(){
		intake = Intake.getInstance();
		wrist = Wrist.getInstance();
		elevator = Elevator.getInstance();
		
		winch = Pigeon.getInstance().getTalon();

		winch.configRemoteFeedbackFilter(Constants.kIsUsingCompBot ? Ports.ELEVATOR_3 : Ports.ELEVATOR_4, RemoteSensorSource.TalonSRX_SelectedSensor, 0, 10);
		winch.configRemoteFeedbackFilter(Constants.kIsUsingCompBot ? Ports.ELEVATOR_3 : Ports.ELEVATOR_4, RemoteSensorSource.Off, 1, 10);
		winch.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 10);
		winch.setNeutralMode(NeutralMode.Brake);
		winch.selectProfileSlot(0, 0);
		winch.config_kP(0, 1.0, 10);
		winch.config_kI(0, 0.0, 10);
		winch.config_kD(0, 0.0, 10);
		winch.config_kF(0, 0.0, 10);
		winch.configAllowableClosedloopError(0, 0, 10);
		
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
	
	private boolean driveTrainFlipped = false;
	public boolean driveTrainFlipped(){ return driveTrainFlipped; }
	
	private void setActiveRequests(RequestList requests){
		activeRequests = requests;
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}
	
	private void clearActiveRequests(){
		setActiveRequests(new RequestList());
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

				if(driveTrainFlipped()){
					if(elevatorHeight < 2.5){
						if(intake.getState() != IntakeState.OFF)
							request(wristIntakeConfig(Constants.kWristHangingAngle, IntakeState.OFF));
					}else if(intake.getState() != IntakeState.OPEN){
						request(intake.stateRequest(IntakeState.OPEN));
					}

					if(elevatorHeight < (Constants.kElevatorMinimumHangingHeight + 0.25)){
						elevator.setManualSpeed(0.25);
					}else{
						elevator.setManualSpeed(0.75);
					}
				}
				
				if(!activeRequestsCompleted){
					if(newRequests){
						if(activeRequests.isParallel()){
							for(Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator.hasNext();){
								Request request = iterator.next();
								request.act();
							}
							newRequests = false;
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
							}else{
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
			/*for(Request r : list.getRequests()){
				addForemostActiveRequest(r);
			}*/
			request(list);
		}
	}
	
	public synchronized void requestWinchOpenLoop(double input){
		if(driveTrainFlipped() && input != 0){
			winch.set(ControlMode.PercentOutput, input);
			winchSetpointSet = false;
		}else if(driveTrainFlipped() && !winchSetpointSet){
			winch.set(ControlMode.Position, winch.getSelectedSensorPosition(0));
			winchSetpointSet = true;
		}else if(!driveTrainFlipped()){
			winch.set(ControlMode.PercentOutput, 0.0);
			winchSetpointSet = false;
		}
	}
	
	/**
	 * Should only be called once per match, when hanging.
	 */
	public synchronized void flipDriveTrain(){
		if(!elevator.isHighGear()){
			elevator.fireLatch(true);
			elevator.setHangingLimits();
			swerve.disable();
			driveTrainFlipped = true;
			DriverStation.reportWarning("Drive train has flipped", false);
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
		SmartDashboard.putNumber("Winch Encoder", winch.getSelectedSensorPosition(0));
	}

}

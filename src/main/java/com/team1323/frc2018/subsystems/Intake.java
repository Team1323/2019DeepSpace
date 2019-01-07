package com.team1323.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1323.frc2018.Constants;
import com.team1323.frc2018.Ports;
import com.team1323.frc2018.loops.ILooper;
import com.team1323.frc2018.loops.Loop;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem{
	private static Intake instance = null;
	public static Intake getInstance(){
		if(instance == null)
			instance = new Intake();
		return instance;
	}
	
	boolean hasCube = false;
	public synchronized boolean hasCube(){
		return hasCube;
	}
	
	private TalonSRX leftIntake, rightIntake;
	private Solenoid pinchers, clampers;
	private DigitalInput banner;
	public boolean getBanner(){
		return banner.get();
	}
	
	private Intake(){
		//leftIntake = TalonSRXFactory.createDefaultTalon(Ports.INTAKE_LEFT);
		//rightIntake = TalonSRXFactory.createDefaultTalon(Ports.INTAKE_RIGHT);
		leftIntake = new TalonSRX(Ports.INTAKE_LEFT);
		rightIntake = new TalonSRX(Ports.INTAKE_RIGHT);
		pinchers = new Solenoid(20, Ports.INTAKE_PINCHERS);
		clampers = new Solenoid(20, Ports.INTAKE_CLAMPERS);
		banner = new DigitalInput(Ports.INTAKE_BANNER);
		
		leftIntake.setInverted(false);
		rightIntake.setInverted(true);
		
		leftIntake.setNeutralMode(NeutralMode.Brake);
		rightIntake.setNeutralMode(NeutralMode.Brake);
		
		leftIntake.configVoltageCompSaturation(12.0, 10);
		rightIntake.configVoltageCompSaturation(12.0, 10);
		leftIntake.enableVoltageCompensation(true);
		rightIntake.enableVoltageCompensation(true);

		leftIntake.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 10);
		leftIntake.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 10);
		rightIntake.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 10);
		rightIntake.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 10);
	}
	
	public void setCurrentLimit(int amps){
		leftIntake.configContinuousCurrentLimit(amps, 10);
		leftIntake.configPeakCurrentLimit(amps, 10);
		leftIntake.configPeakCurrentDuration(10, 10);
		leftIntake.enableCurrentLimit(true);
		rightIntake.configContinuousCurrentLimit(amps, 10);
		rightIntake.configPeakCurrentLimit(amps, 10);
		rightIntake.configPeakCurrentDuration(10, 10);
		rightIntake.enableCurrentLimit(true);
	}

	public void enableCurrentLimit(boolean enable){
		leftIntake.enableCurrentLimit(enable);
		rightIntake.enableCurrentLimit(enable);
	}

	private void setRampRate(double secondsToFull){
		leftIntake.configOpenloopRamp(secondsToFull, 0);
		rightIntake.configOpenloopRamp(secondsToFull, 0);
	}
	
	public enum IntakeState{
		OFF(0,true,false), INTAKING(Constants.kIntakingOutput,true,false), CLAMPING(Constants.kIntakeWeakHoldingOutput,true,true), 
		EJECTING(Constants.kIntakeWeakEjectOutput,true,false), OPEN(0,false,false), INTAKING_WIDE(Constants.kIntakingOutput,false,false), 
		GROUND_CLAMPING(Constants.kIntakeWeakHoldingOutput,true,true), FORCED_INTAKE(Constants.kIntakingOutput,true,false),
		OPEN_EJECTING(-0.3,false,false);
		
		public double leftIntakeOutput = 0;
		public double rightIntakeOutput = 0;
		public boolean pinched = true;
		public boolean clamped = false;
		
		private IntakeState(double output, boolean pinch, boolean clamp){
			this(output, output, pinch, clamp);
		}
		
		private IntakeState(double left, double right, boolean pinch, boolean clamp){
			leftIntakeOutput = left;
			rightIntakeOutput = right;
			pinched = pinch;
			clamped = clamp;
		};
	}
	private IntakeState currentState = IntakeState.OFF;
	public IntakeState getState(){
		return currentState;
	}
	private synchronized void setState(IntakeState newState){
		if(newState != currentState)
			stateChanged = true;
		currentState = newState;
		stateEnteredTimestamp = Timer.getFPGATimestamp();
	}
	private double stateEnteredTimestamp = 0;
	private boolean stateChanged = false;
	
	private double bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
	
	private boolean isResucking = false;
	
	private double holdingOutput = Constants.kIntakeWeakHoldingOutput;
	public void setHoldingOutput(double output){
		holdingOutput = output;
	}
	
	private boolean needsToNotifyDrivers = false;
	public boolean needsToNotifyDrivers(){
		if(needsToNotifyDrivers){
			needsToNotifyDrivers = false;
			return true;
		}
		return false;
	}
	
	public double getAverageCurrent(){
		return (leftIntake.getOutputCurrent() + rightIntake.getOutputCurrent()) / 2.0;
	}
	
	public double getHigherCurrent(){
		double leftCurrent = leftIntake.getOutputCurrent();
		double rightCurrent = rightIntake.getOutputCurrent();
		return Math.max(leftCurrent, rightCurrent);
	}
	
	public void firePinchers(boolean fire){
		pinchers.set(!fire);
	}
	
	public void fireClampers(boolean fire){
		clampers.set(!fire);
	}
	
	private void setRollers(double output){
		setRollers(output, output);
	}
	
	private void setRollers(double left, double right){
		setRampRate(0.0);
		leftIntake.set(ControlMode.PercentOutput, left);
		rightIntake.set(ControlMode.PercentOutput, right);
	}
	
	private void holdRollers(){
		setRollers(holdingOutput);
	}
	
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			hasCube = false;
			needsToNotifyDrivers = false;
			setState(IntakeState.OFF);
			stop();
		}

		@Override
		public void onLoop(double timestamp) {
			switch(currentState){
			case OFF:
				
				break;
			case INTAKING:
				if(stateChanged)
					hasCube = false;
				if(banner.get()){
					if(Double.isInfinite(bannerSensorBeganTimestamp)){
						bannerSensorBeganTimestamp = timestamp;
					}else{
						if(timestamp - bannerSensorBeganTimestamp > /*0.25*/0.1){
							hasCube = true;
							needsToNotifyDrivers = true;
						}
					}
				}else if(!Double.isInfinite(bannerSensorBeganTimestamp)){
					bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
				}
				break;
			case INTAKING_WIDE:
				
				break;
			case FORCED_INTAKE:
				
				break;
			case CLAMPING:
				if(banner.get()){
					if(isResucking){
						holdRollers();
						isResucking = false;
					}
				}else{
					if(!isResucking){
						setRollers(Constants.kIntakingResuckingOutput);
						isResucking = true;
					}
				}
				break;
			case GROUND_CLAMPING:
				
				break;
			case EJECTING:
				if(stateChanged){
					setRampRate(0.0);
					hasCube = false;
				}
				/*if(timestamp - stateEnteredTimestamp > 2.0){
					stop();
					setRampRate(Constants.kIntakeRampRate);
				}*/
				break;
			default:
				break;
			}
			
			if(stateChanged)
				stateChanged = false;
		}

		@Override
		public void onStop(double timestamp) {
			setState(IntakeState.OFF);
			stop();
		}
		
	};
	
	public void eject(double output){
		setState(IntakeState.EJECTING);
		setRollers(output);
		firePinchers(true);
		fireClampers(false);
		hasCube = false;
	}
	
	private void conformToState(IntakeState desiredState){
		setState(desiredState);
		setRollers(desiredState.leftIntakeOutput, desiredState.rightIntakeOutput);
		firePinchers(desiredState.pinched);
		fireClampers(desiredState.clamped);
	}
	
	private void conformToState(IntakeState desiredState, double outputOverride){
		setState(desiredState);
		setRollers(outputOverride);
		firePinchers(desiredState.pinched);
		fireClampers(desiredState.clamped);
	}
	
	public Request stateRequest(IntakeState desiredState) {
		return new Request(){
			
			@Override
			public void act() {
				conformToState(desiredState);
			}
			
		};
	}
	
	public Request waitForCubeRequest(){
		return new Request(){

			@Override
			public void act() {
				conformToState(IntakeState.INTAKING);
			}
			
			@Override
			public boolean isFinished(){
				return !stateChanged && hasCube;
			}
			
		};
	}
	
	public Request ejectRequest(double output){
		return new Request(){
			
			@Override
			public void act() {
				conformToState(IntakeState.EJECTING, output);
			}
			
		};
	}
	
	@Override
	public synchronized void stop(){
		conformToState(IntakeState.OFF);
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
		/*SmartDashboard.putNumber("Left Intake Current", leftIntake.getOutputCurrent());
		SmartDashboard.putNumber("Right Intake Current", rightIntake.getOutputCurrent());
		SmartDashboard.putNumber("Left Intake Voltage", leftIntake.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right Intake Voltage", rightIntake.getMotorOutputVoltage());*/
		SmartDashboard.putBoolean("Intake Has Cube", hasCube);
		SmartDashboard.putBoolean("Intake Banner", banner.get());
	}

}

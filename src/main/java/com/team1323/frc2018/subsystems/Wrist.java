package com.team1323.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.team1323.frc2018.Constants;
import com.team1323.frc2018.Ports;
import com.team1323.frc2018.loops.ILooper;
import com.team1323.frc2018.loops.Loop;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends Subsystem{
	private static Wrist instance = null;
	public static Wrist getInstance(){
		if(instance == null)
			instance = new Wrist();
		return instance;
	}

	LazyTalonSRX wrist;
	private double targetAngle = 0.0;
	private double maxAllowableAngle = Constants.kWristMaxControlAngle;
	public void setMaxAllowableAngle(double angle){
		maxAllowableAngle = angle;
		lockAngle();
	}
	
	public enum WristControlState{
		OPEN_LOOP, POSITION
	}
	private WristControlState currentState = WristControlState.OPEN_LOOP;

	PeriodicIO periodicIO = new PeriodicIO();
	
	private Wrist(){
		//wrist = TalonSRXFactory.createDefaultTalon(Ports.WRIST);
		wrist = new LazyTalonSRX(Ports.WRIST);
		wrist.configVoltageCompSaturation(12.0, 10);
		wrist.enableVoltageCompensation(true);
		wrist.configNominalOutputForward(0.45/12.0, 10);
		wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		wrist.setSensorPhase(false);
		wrist.getSensorCollection().setPulseWidthPosition(0, 100);
		resetToAbsolutePosition();
		wrist.selectProfileSlot(0, 0);
		wrist.config_kP(0, 3.0, 10);
		wrist.config_kI(0, 0.0, 10);
		wrist.config_kD(0, 120.0, 10);
		wrist.config_kF(0, 1023.0/Constants.kWristMaxSpeed, 10);
		wrist.config_kP(1, 3.0, 10);
		wrist.config_kI(1, 0.0, 10);
		wrist.config_kD(1, 240.0, 10);
		wrist.config_kF(1, 1023.0/Constants.kWristMaxSpeed, 10);
		wrist.configMotionCruiseVelocity((int)(Constants.kWristMaxSpeed*1.0), 10);
		wrist.configMotionAcceleration((int)(Constants.kWristMaxSpeed*3.0), 10);
		wrist.configForwardSoftLimitThreshold(wristAngleToEncUnits(Constants.kWristMaxControlAngle), 10);
		wrist.configReverseSoftLimitThreshold(wristAngleToEncUnits(Constants.kWristMinControlAngle), 10);
		wrist.configForwardSoftLimitEnable(true, 10);
		wrist.configReverseSoftLimitEnable(true, 10);
		wrist.configContinuousCurrentLimit(25, 10);
		wrist.configPeakCurrentLimit(30, 10);
		wrist.configPeakCurrentDuration(100, 10);
		wrist.enableCurrentLimit(true);
	}
	
	public void setOpenLoop(double output){
		periodicIO.demand = output;
		currentState = WristControlState.OPEN_LOOP;
	}
	
	public boolean isOpenLoop(){
		return currentState == WristControlState.OPEN_LOOP;
	}
	
	public void setAngle(double angle){
		if(isSensorConnected()){
			if(angle <= maxAllowableAngle){
				targetAngle = angle;
			}else{
				targetAngle = maxAllowableAngle;
			}
			if(angle > getAngle())
				wrist.selectProfileSlot(1, 0);
			else
				wrist.selectProfileSlot(0, 0);
			periodicIO.demand = wristAngleToEncUnits(targetAngle);
			currentState = WristControlState.POSITION;
		}else{
			DriverStation.reportError("Wrist encoder not detected!", false);
			stop();
		}
	}
	
	public void lockAngle(){
		setAngle(getAngle());
	}
	
	public Request angleRequest(double angle){
		return new Request(){

			@Override
			public void act() {
				setAngle(angle);
			}

			@Override
			public boolean isFinished() {
				return hasReachedTargetAngle();
			}
			
		};
	}
	
	public Request lockAngleRequest(){
		return new Request(){
			
			@Override
			public void act(){
				lockAngle();
			}
			
		};
	}
	
	public Request openLoopRequest(double output){
		return new Request(){
			
			@Override
			public void act(){
				setOpenLoop(output);
			}
			
		};
	}
	
	public double getAngle(){
		return encUnitsToWristAngle(periodicIO.position);
	}
	
	public boolean hasReachedTargetAngle(){
		return Math.abs(targetAngle - getAngle()) <= Constants.kWristAngleTolerance;
	}
	
	public double encUnitsToDegrees(double encUnits){
		return encUnits / 4096.0 / Constants.kWristEncoderToOutputRatio * 360.0;
	}
	
	public int degreesToEncUnits(double degrees){
		return (int) (degrees / 360.0 * Constants.kWristEncoderToOutputRatio * 4096.0);
	}
	
	public double encUnitsToWristAngle(int encUnits){
		return Constants.kWristStartingAngle + encUnitsToDegrees(encUnits - Constants.kWristStartingEncoderPosition);
	}
	
	public int wristAngleToEncUnits(double wristAngle){
		return Constants.kWristStartingEncoderPosition + degreesToEncUnits(wristAngle - Constants.kWristStartingAngle);
	}
	
	public boolean isSensorConnected(){
		int pulseWidthPeriod = wrist.getSensorCollection().getPulseWidthRiseToRiseUs();
		return pulseWidthPeriod != 0;
	}
	
	public void resetToAbsolutePosition(){
		int absolutePosition = (int) Util.boundToScope(0, 4096, wrist.getSensorCollection().getPulseWidthPosition());
		if(encUnitsToWristAngle(absolutePosition) > Constants.kWristMaxPhysicalAngle){
			absolutePosition -= 4096;
		}
		double wristAngle = encUnitsToWristAngle(absolutePosition);
		if(wristAngle > Constants.kWristMaxPhysicalAngle || wristAngle < Constants.kWristMinPhysicalAngle){
			DriverStation.reportError("Wrist angle is out of bounds", false);
		}
		wrist.setSelectedSensorPosition(absolutePosition, 0, 10);
	}
	
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			
		}

		@Override
		public void onLoop(double timestamp) {
			if(wrist.getOutputCurrent() > Constants.kWristMaxCurrent){
				//stop();
				DriverStation.reportError("Wrist current high", false);
			}
		}

		@Override
		public void onStop(double timestamp) {
			
		}
		
	};

	@Override
	public synchronized void readPeriodicInputs() {
		periodicIO.position = wrist.getSelectedSensorPosition(0);
		//periodicIO.velocity = wrist.getSelectedSensorVelocity(0);
		//periodicIO.voltage = wrist.getMotorOutputVoltage();
		//periodicIO.current = wrist.getOutputCurrent();
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if(currentState == WristControlState.POSITION)
			wrist.set(ControlMode.MotionMagic, periodicIO.demand);
		else
			wrist.set(ControlMode.PercentOutput, periodicIO.demand);
	}

	@Override
	public void stop() {
		setOpenLoop(0.0);
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
		//SmartDashboard.putNumber("Wrist Current", periodicIO.current);
		//SmartDashboard.putNumber("Wrist Voltage", wrist.getMotorOutputVoltage());
		SmartDashboard.putNumber("Wrist Encoder", periodicIO.position);
		//SmartDashboard.putNumber("Wrist Pulse Width Position", wrist.getSensorCollection().getPulseWidthPosition());
		SmartDashboard.putNumber("Wrist Angle", getAngle());
		//SmartDashboard.putNumber("Wrist Velocity", wrist.getSelectedSensorVelocity(0));
		//SmartDashboard.putNumber("Wrist Error", wrist.getClosedLoopError(0));
		/*if(wrist.getControlMode() == ControlMode.MotionMagic)
			SmartDashboard.putNumber("Wrist Setpoint", wrist.getClosedLoopTarget(0));*/
	}
	
	public boolean checkSystem(){
		double currentMinimum = 0.5;
		double currentMaximum = 20.0;
		
		boolean passed = true;
		
		if(!isSensorConnected()){
			System.out.println("Wrist sensor is not connected, connect and retest");
			return false;
		}
		
		double startingEncPosition = wrist.getSelectedSensorPosition(0);
		wrist.set(ControlMode.PercentOutput, 3.0/12.0);
		Timer.delay(1.0);
		double current = wrist.getOutputCurrent();
		wrist.set(ControlMode.PercentOutput, 0.0);
		if(Math.signum(wrist.getSelectedSensorPosition(0) - startingEncPosition) != 1.0){
			System.out.println("Wrist needs to be reversed");
			passed = false;
		}
		if(current < currentMinimum){
			System.out.println("Wrist current too low: " + current);
			passed = false;
		}else if(current > currentMaximum){
			System.out.println("Wrist current too high: " + current);
			passed = false;
		}
		
		return passed;
	}

	public static class PeriodicIO{
		//Inputs
		public int position;
		public int velocity;
		public double voltage;
		public double current;

		//Outputs
		public double demand;
	}
}

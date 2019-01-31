package com.team1323.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.team1323.frc2018.Constants;
import com.team1323.frc2018.Ports;
import com.team1323.frc2018.loops.ILooper;
import com.team1323.frc2018.loops.Loop;
import com.team1323.frc2018.subsystems.requests.Prerequisite;
import com.team1323.frc2018.subsystems.requests.Request;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.List;

public class Wrist extends Subsystem{
	private static Wrist instance = null;
	public static Wrist getInstance(){
		if(instance == null)
			instance = new Wrist();
		return instance;
	}

	LazyTalonSRX master, slave;
	List<LazyTalonSRX> motors;

	private double targetAngle = 0.0;
	private double maxAllowableAngle = Constants.kWristMaxControlAngle;
	public void setMaxAllowableAngle(double angle){
		maxAllowableAngle = angle;
		lockAngle();
	}

	Solenoid shifter;

	boolean isHighGear = true;
	public boolean isHighGear(){ return isHighGear; }
	boolean highGearConfig = true;

	
	public enum WristControlState{
		OPEN_LOOP, POSITION
	}
	private WristControlState currentState = WristControlState.OPEN_LOOP;

	PeriodicIO periodicIO = new PeriodicIO();
	
	private Wrist(){
		master = new LazyTalonSRX(Ports.WRIST);
		slave = new LazyTalonSRX(Ports.WRIST_2);

		motors = Arrays.asList(master, slave);
		for(LazyTalonSRX talon : motors){
			talon.configVoltageCompSaturation(12.0, 10);
			talon.enableVoltageCompensation(true);
			talon.configNominalOutputForward(0.0/12.0, 10);
			talon.configContinuousCurrentLimit(25, 10);
			talon.configPeakCurrentLimit(30, 10);
			talon.configPeakCurrentDuration(100, 10);
			talon.enableCurrentLimit(true);
		}
		
		master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		master.setSensorPhase(false);
		master.getSensorCollection().setPulseWidthPosition(0, 100);
		resetToAbsolutePosition();
		configForHighGear();
		master.configForwardSoftLimitThreshold(wristAngleToEncUnits(Constants.kWristMaxControlAngle), 10);
		master.configReverseSoftLimitThreshold(wristAngleToEncUnits(Constants.kWristMinControlAngle), 10);
		master.configForwardSoftLimitEnable(true, 10);
		master.configReverseSoftLimitEnable(true, 10);

		setOpenLoop(0.0);
		slave.set(ControlMode.Follower, Ports.WRIST);

		shifter = new Solenoid(20, 0);
	}

	private void configForHighGear(){
		master.selectProfileSlot(0, 0);
		master.config_kP(0, 3.0, 10);
		master.config_kI(0, 0.0, 10);
		master.config_kD(0, 120.0, 10);
		master.config_kF(0, 1023.0/Constants.kWristMaxSpeedHighGear, 10);
		master.config_kP(1, 3.0, 10);
		master.config_kI(1, 0.0, 10);
		master.config_kD(1, 240.0, 10);
		master.config_kF(1, 1023.0/Constants.kWristMaxSpeedHighGear, 10);
		master.configMotionCruiseVelocity((int)(Constants.kWristMaxSpeedHighGear*1.0), 10);
		master.configMotionAcceleration((int)(Constants.kWristMaxSpeedHighGear*3.0), 10);

		highGearConfig = true;
	}

	public void configForLowGear(){
		master.selectProfileSlot(0, 0);
		master.config_kP(0, 3.0, 10);
		master.config_kI(0, 0.0, 10);
		master.config_kD(0, 120.0, 10);
		master.config_kF(0, 1023.0/Constants.kWristMaxSpeedLowGear, 10);
		master.config_kP(1, 3.0, 10);
		master.config_kI(1, 0.0, 10);
		master.config_kD(1, 240.0, 10);
		master.config_kF(1, 1023.0/Constants.kWristMaxSpeedLowGear, 10);
		master.configMotionCruiseVelocity((int)(Constants.kWristMaxSpeedLowGear*1.0), 10);
		master.configMotionAcceleration((int)(Constants.kWristMaxSpeedLowGear*3.0), 10);

		highGearConfig = false;
	}

	public void setHighGear(boolean high){
		if(high && !isHighGear){
			shifter.set(false);
			configForHighGear();
			isHighGear = false;
		}else if(!high && isHighGear){
			shifter.set(true);
			configForLowGear();
			isHighGear = true;
		}
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
			if(isHighGear && !highGearConfig){
				configForHighGear();
			}else if(!isHighGear && highGearConfig){
				configForLowGear();
			}
			if(angle <= maxAllowableAngle){
				targetAngle = angle;
			}else{
				targetAngle = maxAllowableAngle;
			}
			if(angle > getAngle())
				master.selectProfileSlot(1, 0);
			else
				master.selectProfileSlot(0, 0);
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

	public Request angleRequest(double angle, double terminationPoint){
		return new Request(){

			double startingSide;

			@Override
			public void act() {
				setAngle(angle);
				startingSide = Math.signum(terminationPoint - getAngle());
			}

			@Override
			public boolean isFinished() {
				return !Util.epsilonEquals(Math.signum(terminationPoint - getAngle()), startingSide);
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

	public Request gearShiftRequest(boolean high){
		return new Request(){
		
			@Override
			public void act() {
				setHighGear(high);
			}

		};
	}

	public Prerequisite angleReq(double angle, boolean above){
		return new Prerequisite(){
		
			@Override
			public boolean met() {
				return Util.epsilonEquals(Math.signum(angle - getAngle()), above ? -1.0 : 1.0);
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
		int pulseWidthPeriod = master.getSensorCollection().getPulseWidthRiseToRiseUs();
		return pulseWidthPeriod != 0;
	}
	
	public void resetToAbsolutePosition(){
		int absolutePosition = (int) Util.boundToScope(0, 4096, master.getSensorCollection().getPulseWidthPosition());
		if(encUnitsToWristAngle(absolutePosition) > Constants.kWristMaxPhysicalAngle){
			absolutePosition -= 4096;
		}
		double wristAngle = encUnitsToWristAngle(absolutePosition);
		if(wristAngle > Constants.kWristMaxPhysicalAngle || wristAngle < Constants.kWristMinPhysicalAngle){
			DriverStation.reportError("Wrist angle is out of bounds", false);
		}
		master.setSelectedSensorPosition(absolutePosition, 0, 10);
	}
	
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			
		}

		@Override
		public void onLoop(double timestamp) {
			if(master.getOutputCurrent() > Constants.kWristMaxCurrent){
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
		periodicIO.position = master.getSelectedSensorPosition(0);
		if(Constants.kDebuggingOutput){
			periodicIO.velocity = master.getSelectedSensorVelocity(0);
			periodicIO.voltage = master.getMotorOutputVoltage();
			periodicIO.current = master.getOutputCurrent();
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if(currentState == WristControlState.POSITION)
			master.set(ControlMode.MotionMagic, periodicIO.demand);
		else
			master.set(ControlMode.PercentOutput, periodicIO.demand);
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
		SmartDashboard.putNumber("Wrist Angle", getAngle());
		if(Constants.kDebuggingOutput){
			SmartDashboard.putNumber("Wrist Current", periodicIO.current);
			SmartDashboard.putNumber("Wrist Voltage", master.getMotorOutputVoltage());
			SmartDashboard.putNumber("Wrist Encoder", periodicIO.position);
			SmartDashboard.putNumber("Wrist Pulse Width Position", master.getSensorCollection().getPulseWidthPosition());
			SmartDashboard.putNumber("Wrist Velocity", master.getSelectedSensorVelocity(0));
			SmartDashboard.putNumber("Wrist Error", master.getClosedLoopError(0));
			if(master.getControlMode() == ControlMode.MotionMagic)
				SmartDashboard.putNumber("Wrist Setpoint", master.getClosedLoopTarget(0));
		}
	}
	
	public boolean checkSystem(){
		double currentMinimum = 0.5;
		double currentMaximum = 20.0;
		
		boolean passed = true;
		
		if(!isSensorConnected()){
			System.out.println("Wrist sensor is not connected, connect and retest");
			return false;
		}
		
		double startingEncPosition = master.getSelectedSensorPosition(0);
		master.set(ControlMode.PercentOutput, 3.0/12.0);
		Timer.delay(1.0);
		double current = master.getOutputCurrent();
		master.set(ControlMode.PercentOutput, 0.0);
		if(Math.signum(master.getSelectedSensorPosition(0) - startingEncPosition) != 1.0){
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

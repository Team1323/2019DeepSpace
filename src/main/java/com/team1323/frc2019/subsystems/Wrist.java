package com.team1323.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.team1323.frc2019.Constants;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.requests.Prerequisite;
import com.team1323.frc2019.subsystems.requests.Request;
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

	LazyTalonSRX wrist;
	List<LazyTalonSRX> motors;

	private double targetAngle = 0.0;
	private double maxAllowableAngle = Constants.kWristMaxControlAngle;
	public void setMaxAllowableAngle(double angle){
		maxAllowableAngle = angle;
		lockAngle();
	}

	Solenoid shifter;

	boolean isHighGear = false;
	public boolean isHighGear(){ return isHighGear; }
	boolean highGearConfig = false;

	
	public enum WristControlState{
		OPEN_LOOP, POSITION
	}
	private WristControlState currentState = WristControlState.OPEN_LOOP;

	PeriodicIO periodicIO = new PeriodicIO();
	
	private Wrist(){
		wrist = new LazyTalonSRX(Ports.WRIST);

		motors = Arrays.asList(wrist);
		for(LazyTalonSRX talon : motors){
			talon.configVoltageCompSaturation(12.0, 10);
			talon.enableVoltageCompensation(true);
			talon.configNominalOutputForward(0.0/12.0, 10);
			talon.configContinuousCurrentLimit(25, 10);
			talon.configPeakCurrentLimit(30, 10);
			talon.configPeakCurrentDuration(100, 10);
			talon.enableCurrentLimit(true);
		}
		
		wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		wrist.setInverted(false);
		wrist.setSensorPhase(false);
		wrist.getSensorCollection().setPulseWidthPosition(0, 100);
		resetToAbsolutePosition();
		configForHighGear();
		wrist.configForwardSoftLimitThreshold(wristAngleToEncUnits(Constants.kWristMaxControlAngle), 10);
		wrist.configReverseSoftLimitThreshold(wristAngleToEncUnits(Constants.kWristMinControlAngle), 10);
		wrist.configForwardSoftLimitEnable(true, 10);
		wrist.configReverseSoftLimitEnable(true, 10);

		setOpenLoop(0.0);

		shifter = new Solenoid(Ports.DRIVEBASE_PCM, Ports.WRIST_SHIFTER);
	}

	private void configForHighGear(){
		wrist.selectProfileSlot(0, 0);
		wrist.config_kP(0, 1.25, 10); // going down 2.5
		wrist.config_kI(0, 0.0, 10);
		wrist.config_kD(0, 60.0, 10);//80.0
		wrist.config_kF(0, 1023.0/Constants.kWristMaxSpeedHighGear, 10);
		wrist.config_kP(1, 1.25, 10);// going up 2.0
		wrist.config_kI(1, 0.0, 10);
		wrist.config_kD(1, 60.0, 10);//80.0
		wrist.config_kF(1, 1023.0/Constants.kWristMaxSpeedHighGear, 10);
		wrist.configMotionCruiseVelocity((int)(Constants.kWristMaxSpeedHighGear*1.0), 10);
		wrist.configMotionAcceleration((int)(Constants.kWristMaxSpeedHighGear*3.0), 10);
		wrist.configMotionSCurveStrength(6);

		highGearConfig = true;
	}

	public void configForLowGear(){
		wrist.selectProfileSlot(2, 0);
		wrist.config_kP(2, 3.0, 10);
		wrist.config_kI(2, 0.0, 10);
		wrist.config_kD(2, 30.0, 10);
		wrist.config_kF(2, 1023.0/Constants.kWristMaxSpeedLowGear, 10);
		wrist.config_kP(3, 3.0, 10);
		wrist.config_kI(3, 0.0, 10);
		wrist.config_kD(3, 60.0, 10);
		wrist.config_kF(3, 1023.0/Constants.kWristMaxSpeedLowGear, 10);
		wrist.configMotionCruiseVelocity((int)(Constants.kWristMaxSpeedLowGear*1.0), 10);
		wrist.configMotionAcceleration((int)(Constants.kWristMaxSpeedLowGear*3.0), 10);
		wrist.configMotionSCurveStrength(4);

		highGearConfig = false;
		System.out.println("Low gear set");
	}

	public void setHighGear(boolean high){
		if(high && !isHighGear){
			shifter.set(true);
			configForHighGear();
			isHighGear = true;
		}else if(!high && isHighGear){
			shifter.set(false);
			configForLowGear();
			isHighGear = false;
		}
		DriverStation.reportError("Wrist shifted to: " + (high ? "high" : "low"), true);
	}
	
	public void setOpenLoop(double output){
		periodicIO.demand = output * 0.5;//TODO update coefficient
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
				wrist.selectProfileSlot(isHighGear ? 1 : 3, 0);
			else
				wrist.selectProfileSlot(isHighGear ? 0 : 2, 0);
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
				wrist.configMotionCruiseVelocity((int)((isHighGear ? Constants.kWristMaxSpeedHighGear : Constants.kWristMaxSpeedLowGear) * 1.0));
				setAngle(angle);
			}

			@Override
			public boolean isFinished() {
				return hasReachedTargetAngle();
			}
			
		};
	}

	public Request angleRequest(double angle, double speedScalar){
		return new Request(){

			@Override
			public void act() {
				wrist.configMotionCruiseVelocity((int)((isHighGear ? Constants.kWristMaxSpeedHighGear : Constants.kWristMaxSpeedLowGear) * speedScalar));
				setAngle(angle);
			}

			@Override
			public boolean isFinished() {
				return hasReachedTargetAngle();
			}
			
		};
	}

	public Request angleRequest(double angle, double speedScalar, boolean wait){
		return new Request(){

			@Override
			public void act() {
				wrist.configMotionCruiseVelocity((int)((isHighGear ? Constants.kWristMaxSpeedHighGear : Constants.kWristMaxSpeedLowGear) * speedScalar));
				setAngle(angle);
			}

			@Override
			public boolean isFinished() {
				if(wait)
					return hasReachedTargetAngle();
				else
					return true;
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

	public Prerequisite angleRequisite(double angle, boolean above){
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
		int pulseWidthPeriod = wrist.getSensorCollection().getPulseWidthRiseToRiseUs();
		boolean connected = pulseWidthPeriod != 0;
		if(!connected)
			hasEmergency = true;
		return connected;
	}
	
	public void resetToAbsolutePosition(){
		int absolutePosition = (int) Util.boundToScope(0, 4096, wrist.getSensorCollection().getPulseWidthPosition());
		if(encUnitsToWristAngle(absolutePosition) > Constants.kWristMaxPhysicalAngle){
			absolutePosition -= 4096;
		}else if(encUnitsToWristAngle(absolutePosition) < Constants.kWristMinPhysicalAngle){
			absolutePosition += 4096;
		}
		double wristAngle = encUnitsToWristAngle(absolutePosition);
		if(wristAngle > Constants.kWristMaxPhysicalAngle || wristAngle < Constants.kWristMinPhysicalAngle){
			DriverStation.reportError("Wrist angle is out of bounds", false);
			hasEmergency = true;
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
		if(Constants.kDebuggingOutput){
			periodicIO.velocity = wrist.getSelectedSensorVelocity(0);
			periodicIO.voltage = wrist.getMotorOutputVoltage();
			periodicIO.current = wrist.getOutputCurrent();
		}
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
		SmartDashboard.putNumber("Wrist Angle", getAngle());
		if(Constants.kDebuggingOutput){
			SmartDashboard.putNumber("Wrist Current", periodicIO.current);
			SmartDashboard.putNumber("Wrist Voltage", periodicIO.voltage);
			SmartDashboard.putNumber("Wrist Encoder", periodicIO.position);
			SmartDashboard.putNumber("Wrist Pulse Width Position", wrist.getSensorCollection().getPulseWidthPosition());
			SmartDashboard.putNumber("Wrist Velocity", periodicIO.velocity);
			SmartDashboard.putNumber("Wrist Error", encUnitsToDegrees(wrist.getClosedLoopError(0)));
			if(wrist.getControlMode() == ControlMode.MotionMagic)
				SmartDashboard.putNumber("Wrist Setpoint", wrist.getClosedLoopTarget(0));
			SmartDashboard.putBoolean("Wrist High Gear", isHighGear);
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

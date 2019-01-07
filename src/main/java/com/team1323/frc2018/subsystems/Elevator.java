package com.team1323.frc2018.subsystems;

import java.util.Arrays;
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
import com.team254.drivers.LazyTalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem{
	private static Elevator instance = null;
	public static Elevator getInstance(){
		if(instance == null)
			instance = new Elevator();
		return instance;
	}
	
	LazyTalonSRX master, motor2, motor3, motor4;
	List<LazyTalonSRX> motors, slaves;
	Solenoid shifter, latch, forks;
	private double forkDeployTimestamp = Double.POSITIVE_INFINITY;
	private double targetHeight = 0.0;
	private boolean isHighGear = true;
	public boolean isHighGear(){
		return isHighGear;
	}
	private boolean limitsEnabled = false;
	public boolean limitsEnabled(){
		return limitsEnabled;
	}
	
	public TalonSRX getPigeonTalon(){
		return motor2;
	}
	
	public enum ControlState{
		Neutral, Position, OpenLoop, Locked
	}
	private ControlState state = ControlState.Neutral;
	public ControlState getState(){
		return state;
	}
	public void setState(ControlState newState){
		state = newState;
	}
	
	double manualSpeed = Constants.kElevatorTeleopManualSpeed;
	public void setManualSpeed(double speed){
		manualSpeed = speed;
	}

	PeriodicIO periodicIO = new PeriodicIO();
	
	private Elevator(){
		//master = TalonSRXFactory.createDefaultTalon(Ports.ELEVATOR_1);
		//motor2 = TalonSRXFactory.createPermanentSlaveTalon(Ports.ELEVATOR_2, Ports.ELEVATOR_1);
		//motor3 = TalonSRXFactory.createPermanentSlaveTalon(Ports.ELEVATOR_3, Ports.ELEVATOR_1);
		master = new LazyTalonSRX(Ports.ELEVATOR_1);
		motor2 = new LazyTalonSRX(Ports.ELEVATOR_2);
		motor3 = new LazyTalonSRX(Ports.ELEVATOR_3);
		motor4 = new LazyTalonSRX(Ports.ELEVATOR_4);

		motors = Arrays.asList(master, motor2, motor3, motor4);
		slaves = Arrays.asList(motor2, motor3, motor4);

		slaves.forEach((s) -> s.set(ControlMode.Follower, Ports.ELEVATOR_1));
		
		shifter = new Solenoid(20, Ports.ELEVATOR_SHIFTER);
		latch = new Solenoid(20, Ports.ELEVATOR_RELEASE_PISTON);
		forks = new Solenoid(20, Ports.FORKS);
		
		for(LazyTalonSRX motor : motors){
			motor.configVoltageCompSaturation(12.0, 10);
			motor.enableVoltageCompensation(true);
			motor.setNeutralMode(NeutralMode.Brake);
		}
		
		if(Constants.kIsUsingCompBot){
			master.setInverted(false);
			motor2.setInverted(true);
			motor3.setInverted(false);
			motor4.setInverted(true);
		}else{
			master.setInverted(true);
			motor2.setInverted(true);
			motor3.setInverted(true);
			motor4.setInverted(false);
		}
		
		master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		master.setSensorPhase(!Constants.kIsUsingCompBot);
		zeroSensors();
		master.configReverseSoftLimitThreshold(Constants.kElevatorEncoderStartingPosition, 10);
		master.configForwardSoftLimitThreshold(Constants.kElevatorEncoderStartingPosition + feetToEncUnits(Constants.kElevatorMaxHeight), 10);
		master.configForwardSoftLimitEnable(true, 10);
		master.configReverseSoftLimitEnable(true, 10);
		enableLimits(true);
		
		//This sensor is used as a remote for the winch.
		if(Constants.kIsUsingCompBot)
			motor3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		else
			motor4.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		setCurrentLimit(Constants.kELevatorCurrentLimit);
		
		//resetToAbsolutePosition();
		configForLifting();
	}
	
	private void setHighGear(boolean high){
		shifter.set(!high);
		isHighGear = high;
	}
	
	private void configForLifting(){
		setHighGear(true);
		
		manualSpeed = Constants.kElevatorTeleopManualSpeed;

		master.selectProfileSlot(0, 0);
		master.config_kP(0, 1.5, 10);
		master.config_kI(0, 0.0, 10);
		master.config_kD(0, 90.0, 10);
		master.config_kF(0, 1023.0/Constants.kElevatorMaxSpeedHighGear, 10);
		
		if(Constants.kExtraNyooms){
			master.config_kP(1, 1.5, 10);
			master.config_kI(1, 0.0, 10);
			master.config_kD(1, 45.0, 10);//90.0
			master.config_kF(1, 1023.0/Constants.kElevatorMaxSpeedHighGear, 10);
		}else{
			master.config_kP(1, 0.5, 10);
			master.config_kI(1, 0.0, 10);
			master.config_kD(1, 90.0, 10);
			master.config_kF(1, 1023.0/Constants.kElevatorMaxSpeedHighGear, 10);
		}
		
		/*If you decide to go back to full speed on the comp bot, just change the downward PID to match
		the upward PID (as a baseline). Right now it's tuned to be slower, until we can identify
		the source of overshoot and encoder reset.*/
		if(Constants.kExtraNyooms){
			master.configMotionCruiseVelocity((int)(Constants.kElevatorMaxSpeedHighGear * 1.0), 10);
			master.configMotionAcceleration((int)(Constants.kElevatorMaxSpeedHighGear * 1.5), 10);
		}else{
			master.configMotionCruiseVelocity((int)(Constants.kElevatorMaxSpeedHighGear * 0.7), 10);
			master.configMotionAcceleration((int)(Constants.kElevatorMaxSpeedHighGear * 5.0), 10);
		}
	}
	
	public void configForTeleopSpeed(){
		configForLifting();
	}
	
	public void configForAutoSpeed(){
		/*kExtraNyooms might be false, so we have to make sure to set the PID values
		for the faster elevator speed, even if it's redundant.*/
		master.config_kP(0, 1.5, 10);
		master.config_kI(0, 0.0, 10);
		master.config_kD(0, 90.0, 10);
		master.config_kF(0, 1023.0/Constants.kElevatorMaxSpeedHighGear, 10);

		master.config_kP(1, 1.5, 10);
		master.config_kI(1, 0.0, 10);
		master.config_kD(1, 90.0, 10);
		master.config_kF(1, 1023.0/Constants.kElevatorMaxSpeedHighGear, 10);

		master.configMotionCruiseVelocity((int)(Constants.kElevatorMaxSpeedHighGear*1.0), 10);
		master.configMotionAcceleration((int)(Constants.kElevatorMaxSpeedHighGear*3.0), 10);
	}
	
	public void configForHanging(){
		setHighGear(false);
		
		manualSpeed = 1.0;
		
		master.selectProfileSlot(1, 0);
		master.config_kP(1, 8.0, 10);
		master.config_kI(1, 0.0, 10);
		master.config_kD(1, 160.0, 10);
		master.config_kF(1, 1023.0/Constants.kElevatorMaxSpeedLowGear, 10);
		master.configMotionCruiseVelocity((int)(Constants.kElevatorMaxSpeedLowGear*0.9), 10);
		master.configMotionAcceleration((int)(Constants.kElevatorMaxSpeedLowGear*1.0), 10);
	}
	
	public void setHangingLimits(){
		master.configReverseSoftLimitThreshold(feetToEncUnits(Constants.kElevatorMinimumHangingHeight), 10);
		master.configForwardSoftLimitThreshold(feetToEncUnits(Constants.kElevatorMaximumHangingHeight), 10);
		System.out.println("Hanging limits set for the elevator");
	}
	
	public void enableLimits(boolean enable){
		master.overrideSoftLimitsEnable(enable);
		limitsEnabled = enable;
	}
	
	public void setCurrentLimit(int amps){
		for(LazyTalonSRX motor : motors){
			motor.configContinuousCurrentLimit(amps, 10);
			motor.configPeakCurrentLimit(amps, 10);
			motor.configPeakCurrentDuration(10, 10);
			motor.enableCurrentLimit(true);
		}
	}
	
	public void fireLatch(boolean fire){
		latch.set(fire);
	}

	public void fireForks(boolean fire){
		forks.set(fire);
		if(fire) forkDeployTimestamp = Timer.getFPGATimestamp();
	}

	public void toggleForks(){
		forks.set(!forks.get());
	}
	
	public void setOpenLoop(double output){
		setState(ControlState.OpenLoop);
		periodicIO.demand = output * manualSpeed;
	}
	
	public boolean isOpenLoop(){
		return getState() == ControlState.OpenLoop;
	}
	
	public synchronized void setTargetHeight(double heightFeet){
		setState(ControlState.Position);
		if(heightFeet > Constants.kElevatorMaxHeight)
			heightFeet = Constants.kElevatorMaxHeight;
		else if(heightFeet < Constants.kElevatorMinHeight)
			heightFeet = Constants.kElevatorMinHeight;
		if(!isHighGear)
			configForLifting();
		if(isSensorConnected()){
			if(heightFeet > getHeight())
				master.selectProfileSlot(0, 0);
			else
				master.selectProfileSlot(1, 0);
			targetHeight = heightFeet;
			periodicIO.demand = Constants.kElevatorEncoderStartingPosition + feetToEncUnits(heightFeet);
			onTarget = false;
			startTime = Timer.getFPGATimestamp();
		}else{
			DriverStation.reportError("Elevator encoder not detected!", false);
			stop();
		}
	}
	
	public synchronized void setHanigngTargetHeight(double heightFeet){
		setState(ControlState.Position);
		if(isHighGear)
			configForHanging();
		if(isSensorConnected()){
			master.selectProfileSlot(1, 0);
			targetHeight = heightFeet;
			periodicIO.demand = Constants.kElevatorEncoderStartingPosition + feetToEncUnits(heightFeet);
		}else{
			DriverStation.reportError("Elevator encoder not detected!", false);
			stop();
		}
	}
	
	public synchronized void changeHeight(double deltaHeightFeet){
		setState(ControlState.Position);
		if(isSensorConnected()){
			if(deltaHeightFeet > 0)
				master.selectProfileSlot(0, 0);
			else
				master.selectProfileSlot(1, 0);
			periodicIO.demand = master.getSelectedSensorPosition(0) + feetToEncUnits(deltaHeightFeet);
		}else{
			DriverStation.reportError("Elevator encoder not detected!", false);
			stop();
		}
	}
	
	public synchronized void lockHeight(){
		setState(ControlState.Locked);
		if(isSensorConnected()){
			targetHeight = getHeight();
			periodicIO.demand = periodicIO.position;
		}else{
			DriverStation.reportError("Elevator encoder not detected!", false);
			stop();
		}
	}
	
	public Request openLoopRequest(double output){
		return new Request(){
			
			@Override
			public void act(){
				setOpenLoop(output);
			}
			
		};
	}
	
	public Request heightRequest(double height){
		return new Request(){
			
			@Override
			public void act() {
				setTargetHeight(height);
			}

			@Override
			public boolean isFinished() {
				return hasReachedTargetHeight() || isOpenLoop();
			}
			
		};
	}
	
	public Request lowGearHeightRequest(double height){
		return new Request(){
			
			@Override
			public void act() {
				setHanigngTargetHeight(height);
			}

			@Override
			public boolean isFinished() {
				return hasReachedTargetHeight() || isOpenLoop();
			}
			
		};
	}
	
	public Request lockHeightRequest(){
		return new Request(){
			
			@Override
			public void act(){
				lockHeight();
			}
			
		};
	}
	
	public Request gearShiftRequest(boolean high){
		return new Request(){

			@Override
			public void act() {
				if(high) configForLifting();
				else configForHanging();
			}
			
		};
	}
	
	public double getHeight(){
		return encUnitsToFeet(periodicIO.position - Constants.kElevatorEncoderStartingPosition);
	}
	
	public double getVelocityFeetPerSecond(){
		return encUnitsToFeet(periodicIO.velocity) * 10.0;
	}
	
	boolean onTarget = false;
	double startTime = 0.0;
	public boolean hasReachedTargetHeight(){
		if(master.getControlMode() == ControlMode.MotionMagic){
			if((Math.abs(targetHeight - getHeight()) <= Constants.kElevatorHeightTolerance)){
				if(!onTarget){
					System.out.println("Elevator done in: " + (Timer.getFPGATimestamp() - startTime));
					onTarget = true;
				}
				return true;
			}
		}
		return false;
	}
	
	public int feetToEncUnits(double feet){
		return (int) (feet * Constants.kElevatorTicksPerFoot);
	}
	
	public double encUnitsToFeet(double encUnits){
		return encUnits / Constants.kElevatorTicksPerFoot;
	}
	
	private boolean getMotorsWithHighCurrent(){
		return periodicIO.current >= Constants.kElevatorMaxCurrent;
	}
	
	private final Loop loop  = new Loop(){

		@Override
		public void onStart(double timestamp) {
			
		}

		@Override
		public void onLoop(double timestamp) {
			if(getMotorsWithHighCurrent()){
				DriverStation.reportError("Elevator current too high", false);
				//stop();
			}

			if(!Double.isInfinite(forkDeployTimestamp)){
				if(timestamp - forkDeployTimestamp >= 3.0){
					fireForks(false);
					forkDeployTimestamp = Double.POSITIVE_INFINITY;
				}
			}
		}

		@Override
		public void onStop(double timestamp) {
			
		}
		
	};
	
	public boolean isSensorConnected(){
		int pulseWidthPeriod = master.getSensorCollection().getPulseWidthRiseToRiseUs();
		return pulseWidthPeriod != 0;
	}
	
	private void resetToAbsolutePosition(){
		int absolutePosition = master.getSensorCollection().getPulseWidthPosition();
		master.setSelectedSensorPosition(absolutePosition, 0, 10);
	}

	@Override
	public synchronized void readPeriodicInputs(){
		periodicIO.position = master.getSelectedSensorPosition(0);
		//periodicIO.velocity = master.getSelectedSensorVelocity(0);
		//periodicIO.voltage = master.getMotorOutputVoltage();
		//periodicIO.current = master.getOutputCurrent();
	}

	@Override
	public synchronized void writePeriodicOutputs(){
		if(getState() == ControlState.Position || getState() == ControlState.Locked)
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
		master.setSelectedSensorPosition(0, 0, 10);
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	@Override
	public void outputTelemetry() {
		//SmartDashboard.putNumber("Elevator 1 Current", periodicIO.current);
		//SmartDashboard.putNumber("Elevator 2 Current", motor2.getOutputCurrent());
		//SmartDashboard.putNumber("Elevator 3 Current", motor3.getOutputCurrent());
		//SmartDashboard.putNumber("Elevator 4 Current", motor4.getOutputCurrent());
		//SmartDashboard.putNumber("Elevator Voltage", periodicIO.voltage);
		//SmartDashboard.putNumber("Elevator 2 Voltage", motor2.getMotorOutputVoltage());
		//SmartDashboard.putNumber("Elevator 3 Voltage", motor3.getMotorOutputVoltage());
		SmartDashboard.putNumber("Elevator Height", /*Math.round(getHeight()*1000.0)/1000.0*/getHeight());
		//SmartDashboard.putNumber("Elevator Height Graph", getHeight());
		//SmartDashboard.putNumber("Elevator Pulse Width Position", master.getSensorCollection().getPulseWidthPosition());
		SmartDashboard.putNumber("Elevator Encoder", periodicIO.position);
		//SmartDashboard.putNumber("Elevator Velocity", periodicIO.velocity);
		//SmartDashboard.putNumber("Elevator Error", master.getClosedLoopError(0));
		/*if(master.getControlMode() == ControlMode.MotionMagic)
			SmartDashboard.putNumber("Elevator Setpoint", master.getClosedLoopTarget(0));*/
	}
	
	public boolean checkSystem(){
		double currentMinimum = 0.5;
		double currentMaximum = 20.0;
		double timeInterval = 1.0;
		double testOutput = 4.0/12.0;
		double outputDirection = 1.0;
		
		boolean passed = true;
		
		if(!isSensorConnected()){
			System.out.println("Elevator sensor not connected, connect and retest");
			return false;
		}

		configForHanging();
		
		//master.configForwardSoftLimitEnable(false, 10);
		//master.configReverseSoftLimitEnable(false, 10);
		
		motors.forEach((m) -> m.set(ControlMode.PercentOutput, 0.0));
		motors.forEach((m) -> m.setNeutralMode(NeutralMode.Coast));

		Timer.delay(0.25);

		double startingEncPosition = master.getSelectedSensorPosition(0);
		master.set(ControlMode.PercentOutput, testOutput);
		Timer.delay(timeInterval);
		double current = master.getOutputCurrent();
		master.set(ControlMode.PercentOutput, 0.0);
		if(Math.signum(master.getSelectedSensorPosition(0) - startingEncPosition) != 1.0){
			System.out.println("Master elevator motor needs to be reversed");
			passed = false;
		}
		if(current < currentMinimum){
			System.out.println("Master elevator motor current too low: " + current);
			passed = false;
		}else if(current > currentMaximum){
			System.out.println("Master elevator motor current too high: " + current);
			passed = false;
		}else{
			System.out.println("Master elevator motor current good: " + current);
		}
		
		startingEncPosition = master.getSelectedSensorPosition(0);
		motor2.set(ControlMode.PercentOutput, -testOutput);
		Timer.delay(timeInterval);
		current = motor2.getOutputCurrent();
		motor2.set(ControlMode.Follower, Ports.ELEVATOR_1);
		if(Math.signum(master.getSelectedSensorPosition(0) - startingEncPosition) != -1.0){
			System.out.println("Elevator motor 2 needs to be reversed");
			passed = false;
		}
		if(current < currentMinimum){
			System.out.println("Elevator motor 2 current too low: " + current);
			passed = false;
		}else if(current > currentMaximum){
			System.out.println("Elevator motor 2 current too high: " + current);
			passed = false;
		}else{
			System.out.println("Elevator motor 2 current good: " + current);
		}
		
		startingEncPosition = master.getSelectedSensorPosition(0);
		motor3.set(ControlMode.PercentOutput, testOutput);
		Timer.delay(timeInterval);
		current = motor3.getOutputCurrent();
		motor3.set(ControlMode.Follower, Ports.ELEVATOR_1);
		if(Math.signum(master.getSelectedSensorPosition(0) - startingEncPosition) != 1.0){
			System.out.println("Elevator motor 3 needs to be reversed");
			passed = false;
		}
		if(current < currentMinimum){
			System.out.println("Elevator motor 3 current too low: " + current);
			passed = false;
		}else if(current > currentMaximum){
			System.out.println("Elevator motor 3 current too high: " + current);
			passed = false;
		}else{
			System.out.println("Elevator motor 3 current good: " + current);
		}

		startingEncPosition = master.getSelectedSensorPosition(0);
		motor4.set(ControlMode.PercentOutput, -testOutput);
		Timer.delay(timeInterval);
		current = motor4.getOutputCurrent();
		motor4.set(ControlMode.Follower, Ports.ELEVATOR_1);
		if(Math.signum(master.getSelectedSensorPosition(0) - startingEncPosition) != -1.0){
			System.out.println("Elevator motor 4 needs to be reversed");
			passed = false;
		}
		if(current < currentMinimum){
			System.out.println("Elevator motor 4 current too low: " + current);
			passed = false;
		}else if(current > currentMaximum){
			System.out.println("Elevator motor 4 current too high: " + current);
			passed = false;
		}else{
			System.out.println("Elevator motor 4 current good: " + current);
		}

		/*for(LazyTalonSRX motor : motors){
			int index = (motors.indexOf(motor) + 1);
			double startingEncPosition = master.getSelectedSensorPosition(0);
			motor.set(ControlMode.PercentOutput, testOutput*outputDirection);
			Timer.delay(timeInterval / 2.0);
			double current = motor.getOutputCurrent();
			Timer.delay(timeInterval / 2.0);
			motor.set(ControlMode.PercentOutput, 0.0);
			if(Math.signum(master.getSelectedSensorPosition(0) - startingEncPosition) != outputDirection){
				System.out.println("Elevator motor " + index + " needs to be reversed");
				passed = false;
			}
			if(current < currentMinimum){
				System.out.println("Elevator motor " + index + " current too low: " + current);
				passed = false;
			}else if(current > currentMaximum){
				System.out.println("Elevator motor " + index + " current too high: " + current);
				passed = false;
			}else{
				System.out.println("Elevator motor " + index + "current good: " + current);
			}

			outputDirection *= -1.0;
		}*/

		slaves.forEach((s) -> s.set(ControlMode.Follower, Ports.ELEVATOR_1));
		motors.forEach((m) -> m.setNeutralMode(NeutralMode.Brake));
		//master.configForwardSoftLimitEnable(true, 10);
		//master.configReverseSoftLimitEnable(true, 10);
		
		configForLifting();
		
		return passed;
	}

	public static class PeriodicIO{
		//Inputs
		public int position;
		public double velocity;
		public double voltage;
		public double current;

		//outputs
		public double demand;
	}
}

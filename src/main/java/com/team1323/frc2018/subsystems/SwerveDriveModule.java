package com.team1323.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.team1323.frc2018.Constants;
import com.team1323.frc2018.loops.ILooper;
import com.team1323.lib.util.Logger;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonSRX;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveModule extends Subsystem{
	LazyTalonSRX rotationMotor, driveMotor;
	int moduleID;
	String name = "Module ";
	int rotationSetpoint = 0;
	double driveSetpoint = 0;
	int encoderOffset;
	int encoderReverseFactor = 1;
	boolean useDriveEncoder = true;
	boolean tenVoltRotationMode = false;
	private double previousEncDistance = 0;
	private Rotation2d previousWheelAngle = new Rotation2d();
	private Translation2d position;
	private Translation2d startingPosition;
	private Pose2d estimatedRobotPose = new Pose2d();

	PeriodicIO periodicIO = new PeriodicIO();
	
	public SwerveDriveModule(int rotationSlot, int driveSlot, int moduleID, 
			int encoderOffset, Translation2d startingPose){
		//rotationMotor = TalonSRXFactory.createDefaultTalon(rotationSlot);
		//driveMotor = TalonSRXFactory.createDefaultTalon(driveSlot);
		rotationMotor = new LazyTalonSRX(rotationSlot);
		driveMotor = new LazyTalonSRX(driveSlot);
		configureMotors();
		this.moduleID = moduleID;
		name += (moduleID + " ");
		this.encoderOffset = encoderOffset;
		previousEncDistance = 0;
		position = startingPose;
		this.startingPosition = startingPose;
		getRawAngle();
	}
	
	public synchronized void invertDriveMotor(boolean invert){
		driveMotor.setInverted(invert);
	}
	
	public synchronized void invertRotationMotor(boolean invert){
		rotationMotor.setInverted(invert);
	}
	
	public synchronized void reverseDriveSensor(boolean reverse){
		driveMotor.setSensorPhase(reverse);
	}
	
	public synchronized void reverseRotationSensor(boolean reverse){
		encoderReverseFactor = reverse ? -1 : 1;
		rotationMotor.setSensorPhase(reverse);
	}
	
	public synchronized void setNominalDriveOutput(double voltage){
		driveMotor.configNominalOutputForward(voltage / 12.0, 10);
		driveMotor.configNominalOutputReverse(-voltage / 12.0, 10);
	}
	
	public synchronized void setMaxRotationSpeed(double maxSpeed){
		rotationMotor.configMotionCruiseVelocity((int)maxSpeed, 0);
	}

	public synchronized void disableDriveEncoder(){
		useDriveEncoder = false;
	}
	
	private void configureMotors(){
    	rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    	//resetRotationToAbsolute();
    	rotationMotor.setSensorPhase(true);
    	rotationMotor.setInverted(false);
    	rotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
    	rotationMotor.setNeutralMode(NeutralMode.Brake);
		rotationMotor.configVoltageCompSaturation(7.0, 10);
		rotationMotor.enableVoltageCompensation(true);
    	rotationMotor.configAllowableClosedloopError(0, 0, 10);
    	rotationMotor.configMotionAcceleration((int)(Constants.kSwerveRotationMaxSpeed*12.5), 10);
    	rotationMotor.configMotionCruiseVelocity((int)(Constants.kSwerveRotationMaxSpeed), 10);
		rotationMotor.selectProfileSlot(0, 0);
		//Slot 1 is for normal use
    	rotationMotor.config_kP(0, 6.0, 10);//4 8?
    	rotationMotor.config_kI(0, 0.0, 10);
    	rotationMotor.config_kD(0, 160.0, 10);//120 80?
		rotationMotor.config_kF(0, 1023.0/Constants.kSwerveRotationMaxSpeed, 10);
		//Slot 2 is reserved for the beginning of auto
		rotationMotor.config_kP(1, 8.0, 10);
    	rotationMotor.config_kI(1, 0.0, 10);
    	rotationMotor.config_kD(1, 200.0, 10);
    	rotationMotor.config_kF(1, 1023.0/Constants.kSwerveRotation10VoltMaxSpeed, 10);
		rotationMotor.set(ControlMode.MotionMagic, rotationMotor.getSelectedSensorPosition(0));
		if(!isRotationSensorConnected())
			DriverStation.reportError("Module " + moduleID + " rotation encoder not detected!", false);

    	driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    	driveMotor.setSelectedSensorPosition(0, 0, 10);
    	driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
    	driveMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);
    	driveMotor.configVelocityMeasurementWindow(32, 10);
    	driveMotor.configNominalOutputForward(1.5/12.0, 10);//1.5/12.0
    	driveMotor.configNominalOutputReverse(-1.5/12.0, 10);//-1.5/12.0
    	driveMotor.configVoltageCompSaturation(12.0, 10);
    	driveMotor.enableVoltageCompensation(true);
    	driveMotor.configOpenloopRamp(0.25, 10);//0.25
    	driveMotor.configAllowableClosedloopError(0, 0, 10);
    	driveMotor.setInverted(false);
    	driveMotor.setSensorPhase(false);
    	driveMotor.setNeutralMode(NeutralMode.Brake);
    	// Slot 0 is reserved for MotionMagic
    	driveMotor.selectProfileSlot(0, 0);
    	driveMotor.config_kP(0, 2.0, 10);
    	driveMotor.config_kI(0, 0.0, 10);
    	driveMotor.config_kD(0, 24.0, 10);
    	driveMotor.config_kF(0, 1023.0/Constants.kSwerveDriveMaxSpeed, 10);
    	// Slot 1 corresponds to velocity mode
    	driveMotor.config_kP(1, 0.05, 10);
    	driveMotor.config_kI(1, 0.0, 10);
    	driveMotor.config_kD(1, 0.0, 10);
    	driveMotor.config_kF(1, 1023.0/Constants.kSwerveDriveMaxSpeed*0.65, 10);
    	driveMotor.configMotionCruiseVelocity((int)(Constants.kSwerveDriveMaxSpeed*0.9), 10);
		driveMotor.configMotionAcceleration((int)(Constants.kSwerveDriveMaxSpeed), 10);
		if(!isDriveSensorConnected())
			DriverStation.reportError("Module " + moduleID + " drive encoder not detected!", false);
	}

	private boolean isRotationSensorConnected(){
		int pulseWidthPeriod = rotationMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
		return pulseWidthPeriod != 0;
	}

	private boolean isDriveSensorConnected(){
		int pulseWidthPeriod = driveMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
		return pulseWidthPeriod != 0;
	}
	
	private double getRawAngle(){
		return encUnitsToDegrees(periodicIO.rotationPosition);
	}
	
	public Rotation2d getModuleAngle(){
		return Rotation2d.fromDegrees(getRawAngle() - encUnitsToDegrees(encoderOffset));
	}
	
	public Rotation2d getFieldCentricAngle(Rotation2d robotHeading){
		Rotation2d normalizedAngle = getModuleAngle();
		return normalizedAngle.rotateBy(robotHeading);
	}
	
	public void setModuleAngle(double goalAngle){
		double newAngle = Util.placeInAppropriate0To360Scope(getRawAngle(), goalAngle + encUnitsToDegrees(encoderOffset));
		int setpoint = degreesToEncUnits(newAngle);
		periodicIO.rotationControlMode = ControlMode.MotionMagic;
		periodicIO.rotationDemand = setpoint;
	}
	
	public boolean angleOnTarget(){
		double error = encUnitsToDegrees(Math.abs(periodicIO.rotationDemand - periodicIO.rotationPosition));
		return error < 4.5;
	}
	
	public void set10VoltRotationMode(boolean tenVolts){
		if(tenVolts && !tenVoltRotationMode){
			rotationMotor.selectProfileSlot(1, 0);
			rotationMotor.configVoltageCompSaturation(10.0, 10);
			tenVoltRotationMode = true;
		}else if(!tenVolts && tenVoltRotationMode){
			rotationMotor.selectProfileSlot(0, 0);
			rotationMotor.configVoltageCompSaturation(7.0, 10);
			tenVoltRotationMode = false;
		}
	}

	public void setRotationOpenLoop(double power){
		periodicIO.rotationControlMode = ControlMode.PercentOutput;
		periodicIO.rotationDemand = power;
	}
	
	/**
	 * @param velocity Normalized value
	 */
	public void setDriveOpenLoop(double velocity){
		double volts = 0.0;
		if(!Util.epsilonEquals(velocity, 0.0, Constants.kEpsilon)){
			velocity *= Constants.kSwerveMaxSpeedFeetPerSecond;
			double m =  Constants.kVoltageVelocityEquations[moduleID][velocity < 0 ? 1 : 0][0];
			double b = Constants.kVoltageVelocityEquations[moduleID][velocity < 0 ? 1 : 0][1];
			volts = (velocity - b) / m;
			volts = Util.deadBand(volts, 1.0);
		}

		periodicIO.driveControlMode = ControlMode.PercentOutput;
		periodicIO.driveDemand = volts / 12.0;
	}
	
	public void setDrivePositionTarget(double deltaDistanceInches){
		driveMotor.selectProfileSlot(0, 0);
		periodicIO.driveControlMode = ControlMode.MotionMagic;
		periodicIO.driveDemand = periodicIO.drivePosition + inchesToEncUnits(deltaDistanceInches);
	}
	
	public boolean drivePositionOnTarget(){
		if(driveMotor.getControlMode() == ControlMode.MotionMagic)
			return encUnitsToInches((int)Math.abs(periodicIO.driveDemand - periodicIO.drivePosition)) < 2.0;
		return false;
	}
	
	public void setVelocitySetpoint(double feetPerSecond){
		driveMotor.selectProfileSlot(1, 0);
		periodicIO.driveControlMode = ControlMode.Velocity;
		periodicIO.driveDemand = feetPerSecondToEncVelocity(feetPerSecond);
	}
	
	private double getDriveDistanceFeet(){
		return getDriveDistanceInches() / 12.0;
	}
	
	private double getDriveDistanceInches(){
		return encUnitsToInches(periodicIO.drivePosition);
	}
	
	public double encUnitsToInches(double encUnits){
		return encUnits/Constants.kSwerveEncUnitsPerInch;
	}
	
	public int inchesToEncUnits(double inches){
		return (int) (inches*Constants.kSwerveEncUnitsPerInch);
	}
	
	public double encVelocityToFeetPerSecond(double encUnitsPer100ms){
		return encUnitsToInches(encUnitsPer100ms) / 12.0 * 10;
	}
	
	public int feetPerSecondToEncVelocity(double feetPerSecond){
		return (int) (inchesToEncUnits(feetPerSecond * 12.0 / 10.0));
	}
	
	public int degreesToEncUnits(double degrees){
		return (int) (degrees/360.0*Constants.kSwerveDriveEncoderResolution);
	}
	
	public double encUnitsToDegrees(double encUnits){
		return encUnits/Constants.kSwerveDriveEncoderResolution*360.0;
	}
	
	public Translation2d getPosition(){
		return position;
	}
	
	public Pose2d getEstimatedRobotPose(){
		return estimatedRobotPose;
	}
	
	public synchronized void updatePose(Rotation2d robotHeading){
		double currentEncDistance = getDriveDistanceFeet();
		double deltaEncDistance = (currentEncDistance - previousEncDistance) * Constants.kWheelScrubFactors[moduleID];
		Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
		Translation2d deltaPosition = new Translation2d(currentWheelAngle.cos()*deltaEncDistance, 
				currentWheelAngle.sin()*deltaEncDistance);
		deltaPosition = new Translation2d(deltaPosition.x() * ((Math.signum(deltaPosition.x()) == -1.0) ? 1.0 : Constants.kXScrubFactor),
			deltaPosition.y() * ((Math.signum(deltaPosition.y()) == -1.0) ? 1.0 : Constants.kYScrubFactor));
		Translation2d updatedPosition = position.translateBy(deltaPosition);
		Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
		Pose2d robotPose = staticWheelPose.transformBy(Pose2d.fromTranslation(startingPosition).inverse());
		position = updatedPosition;
		estimatedRobotPose =  robotPose;
		previousEncDistance = currentEncDistance;
		previousWheelAngle = currentWheelAngle;
	}
	
	public synchronized void resetPose(Pose2d robotPose){
		Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(startingPosition)).getTranslation();
		position = modulePosition;
	}
	
	public synchronized void resetPose(){
		position = startingPosition;
	}
	
	public synchronized void resetLastEncoderReading(){
		previousEncDistance = getDriveDistanceFeet();
	}

	@Override
	public synchronized void readPeriodicInputs() {
		periodicIO.rotationPosition = rotationMotor.getSelectedSensorPosition(0);
		if(useDriveEncoder) periodicIO.drivePosition = driveMotor.getSelectedSensorPosition(0);
		/*if(moduleID == 3){
			periodicIO.velocity = driveMotor.getSelectedSensorVelocity(0);
			periodicIO.driveVoltage = driveMotor.getMotorOutputVoltage();
			if(periodicIO.velocity != 0 && periodicIO.driveVoltage != 0)
				Logger.log("(" + periodicIO.driveVoltage + ", " + encVelocityToFeetPerSecond(periodicIO.velocity) + "), ");
		}*/
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		rotationMotor.set(periodicIO.rotationControlMode, periodicIO.rotationDemand);
		driveMotor.set(periodicIO.driveControlMode, periodicIO.driveDemand);
	}
	
	@Override
	public synchronized void stop(){
		setDriveOpenLoop(0.0);
	}
	
	public synchronized void disable(){
		setDriveOpenLoop(0.0);
		setRotationOpenLoop(0.0);
	}
	
	public synchronized void resetRotationToAbsolute(){
		rotationMotor.setSelectedSensorPosition(
				encoderReverseFactor * (rotationMotor.getSensorCollection().getPulseWidthPosition() - encoderOffset), 0, 10);
	}

	@Override
	public synchronized void zeroSensors() {
		zeroSensors(new Pose2d());
	}
	
	public synchronized void zeroSensors(Pose2d robotPose) {
		driveMotor.setSelectedSensorPosition(0, 0, 0);
		//resetRotationToAbsolute();
		resetPose(robotPose);
		estimatedRobotPose = robotPose;
		previousEncDistance = 0;
		previousWheelAngle = getFieldCentricAngle(robotPose.getRotation());
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putNumber(name + "Angle", getModuleAngle().getDegrees());
		//SmartDashboard.putNumber(name + "Pulse Width", rotationMotor.getSelectedSensorPosition(0));
		//SmartDashboard.putNumber(name + "Drive Voltage", periodicIO.driveVoltage);
		SmartDashboard.putNumber(name + "Inches Driven", getDriveDistanceInches());
		//SmartDashboard.putNumber(name + "Rotation Voltage", rotationMotor.getMotorOutputVoltage());
		//SmartDashboard.putNumber(name + "Velocity", encVelocityToFeetPerSecond(periodicIO.velocity));
		/*if(rotationMotor.getControlMode() == ControlMode.MotionMagic)
			SmartDashboard.putNumber(name + "Error", encUnitsToDegrees(rotationMotor.getClosedLoopError(0)));*/
		//SmartDashboard.putNumber(name + "X", position.x());
		//SmartDashboard.putNumber(name + "Y", position.y());
		//SmartDashboard.putNumber(name + "Drive Current", driveMotor.getOutputCurrent());
		//SmartDashboard.putNumber(name + "Rotation Speed", rotationMotor.getSelectedSensorVelocity(0));
	}

	public static class PeriodicIO{
		//Inputs
		public int rotationPosition;
		public int drivePosition;
		public int velocity;
		public double driveVoltage;
		

		//Outputs
		public ControlMode rotationControlMode = ControlMode.PercentOutput;
		public ControlMode driveControlMode = ControlMode.PercentOutput;
		public double rotationDemand;
		public double driveDemand;
	}
	
}

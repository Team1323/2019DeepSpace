/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.team1323.frc2019.Constants;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team254.drivers.LazyTalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Jacks extends Subsystem {
    private static Jacks instance = null;
    public static Jacks getInstance(){
        if(instance == null)
            instance = new Jacks();
        return instance;
    }

    LazyTalonSRX motor;
    public LazyTalonSRX getPigeonTalon(){
        return motor;
    }

    PeriodicIO periodicIO = new PeriodicIO();

    double targetHeight = 0.0;

    public Jacks(){
        motor = new LazyTalonSRX(Ports.JACKS);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        motor.setInverted(false);
        motor.setSensorPhase(false);

        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);

        motor.config_kP(0, 0.1);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.0);
        motor.config_kF(0, 1023.0 / Constants.kJackMaxSpeed);
        motor.configMotionCruiseVelocity((int)(Constants.kJackMaxSpeed * 1.0));
        motor.configMotionAcceleration((int)(Constants.kJackMaxSpeed * 1.0));

        setOpenLoop(0.0);
    }

    public enum ControlState{
        OPEN_LOOP, POSITION
    }
    ControlState state = ControlState.OPEN_LOOP;
    public ControlState getState(){ return state; }

    public boolean isOpenLoop(){
        return state == ControlState.OPEN_LOOP;
    }

    public synchronized void setOpenLoop(double percentOutput){
        state = ControlState.OPEN_LOOP;
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.setpoint = percentOutput;
    }

    public synchronized void setHeight(double height){
        state = ControlState.POSITION;
        periodicIO.controlMode = ControlMode.MotionMagic;
        periodicIO.setpoint = jackHeightToEncUnits(height);
        targetHeight = height;
    }

    public synchronized void lockHeight(){
        setHeight(getHeight());
    }

    public Request openLoopRequest(double input){
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(input);
            }
        };
    }

    public Request heightRequest(double height){
        return new Request(){
        
            @Override
            public void act() {
                setHeight(height);    
            }

            @Override
            public boolean isFinished(){
                return hasReachedTargetHeight() || isOpenLoop();
            }

        };
    }

    public Request lockHeightRequest(){
        return new Request(){
        
            @Override
            public void act() {
                lockHeight();
            }
        };
    }

    public double getHeight(){
        return encUnitsToJackHeight(periodicIO.position);
    }

    public boolean hasReachedTargetHeight(){
        return Math.abs(targetHeight - getHeight()) <= Constants.kJackHeightTolerance;
    }

    private double encUnitsToInches(double encUnits){
        return encUnits / Constants.kJackTicksPerInch;
    }

    private double inchesToEncUnits(double inches){
        return inches * Constants.kJackTicksPerInch;
    }

    private double encUnitsToJackHeight(double encUnits){
        return encUnitsToInches(encUnits - Constants.kJackStartingEncPosition);
    }

    private double jackHeightToEncUnits(double jackHeight){
        return Constants.kJackStartingEncPosition + inchesToEncUnits(jackHeight);
    }
    
    @Override
    public void readPeriodicInputs() {
        periodicIO.position = motor.getSelectedSensorPosition();
        if(Constants.kDebuggingOutput){
            periodicIO.velocity = motor.getSelectedSensorVelocity();
            periodicIO.voltage = motor.getMotorOutputVoltage();
            periodicIO.current = motor.getOutputCurrent();
        }
    }

    @Override
    public void writePeriodicOutputs() {
        motor.set(periodicIO.controlMode, periodicIO.setpoint);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Jack Height", getHeight());
        if(Constants.kDebuggingOutput){
            SmartDashboard.putNumber("Jack Velocity", periodicIO.velocity);
            SmartDashboard.putNumber("Jack Voltage", periodicIO.voltage);
            SmartDashboard.putNumber("Jack Current", periodicIO.current);
            SmartDashboard.putNumber("Jack Encoder", periodicIO.position);
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public static class PeriodicIO{
        //inputs
        public int position = 0;
        public int velocity = 0;
        public double voltage = 0.0;
        public double current = 0.0;

        //outputs
        public double setpoint = 0.0;
        public ControlMode controlMode = ControlMode.PercentOutput;
    }
}

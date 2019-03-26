/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.frc2019.Constants;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team254.drivers.LazyTalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 */
public class BallCarriage extends Subsystem{
    private static BallCarriage instance = null;
    public static BallCarriage getInstance(){
        if(instance == null)
            instance = new BallCarriage();
        return instance;
    }

    LazyTalonSRX motor;
    DigitalInput banner;

    private boolean getBanner(){
        return banner.get();
    }

    public BallCarriage(){
        motor = new LazyTalonSRX(Ports.BALL_CARRIAGE);
        banner = new DigitalInput(Ports.BALL_CARRIAGE_BANNER);
        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
        setCurrentLimit(60);//20
        setOpenLoop(0.0);
    }

    public void setCurrentLimit(int amps){
        motor.configPeakCurrentLimit(amps);
        motor.configPeakCurrentDuration(10);
        motor.configContinuousCurrentLimit(amps);
        motor.enableCurrentLimit(true);
    }

    public enum State{
        EJECTING(Constants.kBallCarriageEjectOutput), 
        RECEIVING(Constants.kBallCarriageReceiveOutput), 
        SUCKING(Constants.kBallCarriageSuckOutput),
        OFF(0.0);

        double output = 0.0;
        private State(double output){
            this.output = output;
        }
    }
    private State currentState = State.OFF;
    public State getState(){ return currentState; }
    private synchronized void setState(State newState) {
        if (newState != currentState)
          stateChanged = true;
        currentState = newState;
        stateEnteredTimestamp = Timer.getFPGATimestamp();
    }

    private boolean stateChanged = false;
    private double bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
    private double stateEnteredTimestamp = 0;

    private boolean hasBall = false;
    public boolean hasBall(){ return hasBall; }

    private boolean needsToNotifyDrivers = false;
    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
        needsToNotifyDrivers = false;
        return true;
        }
        return false;
    }

    public synchronized void setOpenLoop(double percentOutput){
        motor.set(ControlMode.PercentOutput, percentOutput);
    }

    private final Loop loop = new Loop(){
    
        @Override
        public void onStart(double timestamp) {
            setState(State.OFF);
        }
    
        @Override
        public void onLoop(double timestamp) {
            switch(currentState){
                case EJECTING:
                    if(stateChanged)
                        hasBall = false;
                    if((timestamp - stateEnteredTimestamp) > 1.0){
                        conformToState(State.OFF);
                    }
                    break;
                case RECEIVING:
                    if (stateChanged){
                        hasBall = false;
                        bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
                    }
                    if (getBanner()) {
                        if (Double.isInfinite(bannerSensorBeganTimestamp)) {
                            bannerSensorBeganTimestamp = timestamp;
                        } else {
                            if ((timestamp - bannerSensorBeganTimestamp) >= 0.0) {
                                hasBall = true;
                                needsToNotifyDrivers = true;
                                conformToState(State.OFF);
                            }
                        }
                    } else {
                        if (!Double.isFinite(bannerSensorBeganTimestamp)) {
                            bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
                        }
                    }
                    break;
                case SUCKING:
                    if(banner.get()){
                        conformToState(State.OFF);
                    }
                    break;
                default:
                    break;
            }
            if(stateChanged)
                stateChanged = false;
        }

        @Override
        public void onStop(double timestamp) {
            setState(State.OFF);
        }
    };

    public void conformToState(State newState){
        setOpenLoop(newState.output);
        setState(newState);
    }

    public void conformToState(State newState, double outputOverride){
        setOpenLoop(outputOverride);
        setState(newState);
    }

    public Request stateRequest(State newState){
        return new Request(){
        
            @Override
            public void act() {
                conformToState(newState);    
            }

        };
    }

    public Request waitForBallRequest(){
        return new Request(){

            @Override
            public void act(){

            }

            @Override
            public boolean isFinished(){
                return hasBall() && !stateChanged;
            }

        };
    }

    @Override
    public void outputTelemetry() {
        if(Constants.kDebuggingOutput){
            SmartDashboard.putNumber("Ball Carriage Voltage", motor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Ball Carriage Current", motor.getOutputCurrent());
            SmartDashboard.putBoolean("Ball Carriage Banner", getBanner());
            SmartDashboard.putBoolean("Ball Carriage Has Ball", hasBall());
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import com.team1323.frc2019.Ports;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.requests.Request;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team1323.frc2019.Constants;

public class Probe extends Subsystem {
    private static Probe instance = null;
    public static Probe getInstance(){
        if(instance == null)
            instance = new Probe();
        return instance;
    }

    Solenoid fingers, scorer, extender;
    DigitalInput banner;

    private boolean getBanner(){
        return banner.get();
    }

    public Probe(){
        extender = new Solenoid(Ports.CARRIAGE_PCM, Ports.PROBE_EXTENDER);
        scorer = new Solenoid(Ports.CARRIAGE_PCM, Ports.PROBE_SCORER);
        fingers = new Solenoid(Ports.CARRIAGE_PCM, Ports.PROBE_FINGERS);

        banner = new DigitalInput(Ports.DISK_INTAKE_BANNER);
    }

    public enum State{
        SCORING(true, true, false), STOWED(false, false, true),
        HOLDING(true, true, true), RECEIVING(true, false, true),
        STOWED_HOLDING(false, true, true),
        GROUND_INTAKING(false, false, true),
        NEUTRAL_EXTENDED(true, false, true);

        boolean extended;
        boolean scoring;
        boolean fingers;

        private State(boolean extended, boolean scoring, boolean fingers){
            this.extended = extended;
            this.scoring = scoring;
            this.fingers = fingers;
        }
    }
    private State currentState = State.STOWED;
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

    private boolean hasDisk = false;
    public boolean hasDisk(){ return hasDisk; }
    public void feignDisk(){ hasDisk = true; }

    private boolean needsToNotifyDrivers = false;
    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
        needsToNotifyDrivers = false;
        return true;
        }
        return false;
    }

    public void conformToState(State newState){
        extender.set(newState.extended);
        scorer.set(newState.scoring);
        fingers.set(!newState.fingers);
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

    public Request waitForDiskRequest(){
        return new Request(){

            @Override
            public void act(){

            }

            @Override
            public boolean isFinished(){
                return hasDisk() && !stateChanged;
            }

        };
    }

    private final Loop loop = new Loop(){
    
        @Override
        public void onStart(double timestamp) {
            
        }
    
        @Override
        public void onLoop(double timestamp) {
            switch(currentState){
                case SCORING:
                    break;
                case STOWED:
                    break;
                case HOLDING:
                    break;
                case RECEIVING:
                    if (stateChanged)
                        hasDisk = false;
                    if (getBanner()) {
                        if (Double.isInfinite(bannerSensorBeganTimestamp)) {
                            bannerSensorBeganTimestamp = timestamp;
                        } else {
                            if (timestamp - bannerSensorBeganTimestamp >= 0.0) {
                                hasDisk = true;
                                needsToNotifyDrivers = true;
                            }
                        }
                    } else if (!Double.isFinite(bannerSensorBeganTimestamp)) {
                        bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
                    }
                    break;
                case GROUND_INTAKING:
                    if (stateChanged)
                        hasDisk = false;
                    if (getBanner()) {
                        if (Double.isInfinite(bannerSensorBeganTimestamp)) {
                            bannerSensorBeganTimestamp = timestamp;
                        } else {
                            if (timestamp - bannerSensorBeganTimestamp >= 0.0) {
                                hasDisk = true;
                                needsToNotifyDrivers = true;
                            }
                        }
                    } else if (!Double.isFinite(bannerSensorBeganTimestamp)) {
                        bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
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
            
        }

    };

    @Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}

    @Override
    public void outputTelemetry() {
        if(Constants.kDebuggingOutput){
            SmartDashboard.putBoolean("Probe Has Disk", hasDisk());
            SmartDashboard.putBoolean("Probe Banner", getBanner());
        }
    }

    @Override
    public void stop() {

    }

}

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

import edu.wpi.first.wpilibj.Solenoid;

public class Probe extends Subsystem {
    private static Probe instance = null;
    public static Probe getInstance(){
        if(instance == null)
            instance = new Probe();
        return instance;
    }

    Solenoid fingers, scorer, extender;

    public Probe(){
        extender = new Solenoid(Ports.CARRIAGE_PCM, Ports.PROBE_EXTENDER);
        scorer = new Solenoid(Ports.CARRIAGE_PCM, Ports.PROBE_SCORER);
        fingers = new Solenoid(Ports.CARRIAGE_PCM, Ports.PROBE_FINGERS);
    }

    public enum State{
        SCORING(true, true, false), STOWED(false, false, true),
        HOLDING(true, true, true), RECEIVING(true, false, true);

        boolean extended;
        boolean scoring;
        boolean fingers;

        private State(boolean extended, boolean scoring, boolean fingers){
            this.extended = extended;
            this.scoring = scoring;
            this.fingers = fingers;
        }
    }
    State state = State.STOWED;
    public State getState(){
        return state;
    }

    private void conformToState(State newState){
        extender.set(newState.extended);
        scorer.set(newState.scoring);
        fingers.set(!newState.fingers);
    }

    public Request stateRequest(State newState){
        return new Request(){
        
            @Override
            public void act() {
                conformToState(newState);
            }

        };
    }

    private final Loop loop = new Loop(){
    
        @Override
        public void onStart(double timestamp) {
            
        }
    
        @Override
        public void onLoop(double timestamp) {
            switch(state){
                case SCORING:
                break;
                case STOWED:
                break;
                case HOLDING:
                break;
                case RECEIVING:
                break;
                default:
                break;
            }
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

    }

    @Override
    public void stop() {

    }

}

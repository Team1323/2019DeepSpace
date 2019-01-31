/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2018.subsystems;

import com.team1323.frc2018.Ports;
import com.team1323.frc2018.loops.ILooper;
import com.team1323.frc2018.loops.Loop;

import edu.wpi.first.wpilibj.Solenoid;

public class Probe extends Subsystem {
    private Probe instance = null;
    public Probe getInstance(){
        if(instance == null)
            instance = new Probe();
        return instance;
    }

    Solenoid fingers, score1, score2, extender1, extender2;

    public Probe(){
        extender1 = new Solenoid(20, Ports.PROBE_EXTENDER_1);
        extender2 = new Solenoid(20, Ports.PROBE_EXTENDER_2);
        score1 = new Solenoid(20, Ports.PROBE_SCORER_1);
        score2 = new Solenoid(20, Ports.PROBE_SCORER_2);
        fingers = new Solenoid(20, Ports.PROBE_FINGERS);
    }

    public enum State{
        SCORING(true, true, false);

        boolean extended;
        boolean scoring;
        boolean fingers;

        private State(boolean extended, boolean scoring, boolean fingers){
            this.extended = extended;
            this.scoring = scoring;
            this.fingers = fingers;
        }
    }
    State state = State.SCORING;
    public State getState(){
        return state;
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

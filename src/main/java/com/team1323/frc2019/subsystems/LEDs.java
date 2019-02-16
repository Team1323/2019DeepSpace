/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import java.sql.Time;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.requests.Request;

import edu.wpi.first.wpilibj.Timer;

/**
 * 
 */
public class LEDs extends Subsystem{
    private static LEDs instance = null;
    public static LEDs getInstance(){
        if(instance == null)
            instance = new LEDs();
        return instance;
    }

    CANifier canifier;

    public LEDs(){
        canifier = new CANifier(Ports.CANIFIER);
    }

    boolean lit = false;
    double lastOnTime = 0.0;
    double lastOffTime = 0.0;

    public enum State{
        OFF(0.0, 0.0, 0.0, Double.POSITIVE_INFINITY, 0.0),
        DISABLED(1.0, 0.2, 0.2, Double.POSITIVE_INFINITY, 0.0),
        ENABLED(0.0, 0.0, 1.0, Double.POSITIVE_INFINITY, 0.0),
        EMERGENCY(1.0, 0.0, 0.0, 0.5, 0.5);

        double red, green, blue, onTime, offTime;
        private State(double r, double g, double b, double onTime, double offTime){
            red = r;
            green = g;
            blue = b;
            this.onTime = onTime;
            this.offTime = offTime;
        }
    }
    private State currentState = State.OFF;
    public State getState(){ return currentState; }
    private void setState(State newState){
        if(newState != currentState){
            currentState = newState;
            lastOffTime = 0.0;
            lastOnTime = 0.0;
            lit = false;
        }
    }

    private final Loop loop = new Loop(){

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            
        }

        @Override
        public void onStop(double timestamp) {

        }

    };


    public void setLEDs(double r, double g, double b){
		//A: Green
		//B: Red
		//C: Blue
		canifier.setLEDOutput(r, LEDChannel.LEDChannelB);
		canifier.setLEDOutput(g, LEDChannel.LEDChannelA);
		canifier.setLEDOutput(b, LEDChannel.LEDChannelC);
    }

    public void conformToState(State state){
        setState(state);
    }
    
    public Request colorRequest(State newState){
        return new Request(){
        
            @Override
            public void act() {
                
            }
        };
    }

    @Override
    public void writePeriodicOutputs(){
        double timestamp = Timer.getFPGATimestamp();
        if(!lit && (timestamp - lastOffTime) >= currentState.offTime){
            setLEDs(currentState.red, currentState.green, currentState.blue);
            lastOnTime = timestamp;
            lit = true;
        }else if(lit && !Double.isInfinite(currentState.onTime)){
            if((timestamp - lastOnTime) >= currentState.onTime){
                setLEDs(0.0, 0.0, 0.0);
                lastOffTime = timestamp;
                lit = false;
            }
        }
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper){
        enabledLooper.register(loop);
    }

    @Override
    public void stop() {

    }
}

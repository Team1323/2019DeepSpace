/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.subsystems.requests.Request;

/**
 * Add your docs here.
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

    double onTime = 1.0;
    double offTime = 0.0;

    public enum State{
        OFF(0.0, 0.0, 0.0, 1.0, 0.0),
        DISABLED(1.0, 0.2, 0.2, 1.0, 0.0),
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
        currentState = newState;
    }


    public void setLEDs(double r, double g, double b){
		//A: Green
		//B: Red
		//C: Blue
		canifier.setLEDOutput(r, LEDChannel.LEDChannelB);
		canifier.setLEDOutput(g, LEDChannel.LEDChannelA);
		canifier.setLEDOutput(b, LEDChannel.LEDChannelC);
    }

    public void setIntervals(double onTime, double offTime){
        this.onTime = onTime;
        this.offTime = offTime;
    }

    public void conformToState(State state){
        setLEDs(state.red, state.green, state.blue);
        setIntervals(state.onTime, state.offTime);
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
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team1323.frc2019.Constants;
import com.team1323.lib.util.HSVtoRGB;
import com.team1323.lib.util.MovingAverage;

import edu.wpi.first.wpilibj.Timer;

/**
 * Brings all da colors to da club
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
    double transTime = 0.0;

    public enum State{
        OFF(0.0, 0.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        DISABLED(255.0, 20.0, 30.0, Double.POSITIVE_INFINITY, 0.0, false),
        ENABLED(0.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false),
        EMERGENCY(255.0, 0.0, 0.0, 0.5, 0.5, false),
        BALL_IN_INTAKE(255.0, 20.0, 0.0, 0.5, 0.5, false),
        BALL_IN_CARRIAGE(255.0, 20.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        DISK_IN_INTAKE(255.0, 60.0, 0.0, 0.5, 0.5, false),
        DISK_IN_PROBE(255.0, 60.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        TARGET_VISIBLE(0.0, 255.0, 0.0, Double.POSITIVE_INFINITY, 0.0, false),
        TARGET_TRACKING(0.0, 255.0, 0.0, 0.0625, 0.0625, false),
        CLIMBING(255.0, 0.0, 255.0, Double.POSITIVE_INFINITY, 0.0, false),
        RAINBOW(0, true),
        BREATHING_PINK(342, 1.0, true);

        double red, green, blue, onTime, offTime, cycleTime, transitionTime;
        float startingHue;
        List<List<Double>> colors = new ArrayList<List<Double>>();
        boolean isCycleColors;
        private State(double r, double g, double b, double onTime, double offTime, boolean isCycleColors){
            red = r / 255.0;
            green = g / 255.0;
            blue = b / 255.0;
            this.onTime = onTime;
            this.offTime = offTime;
        }

        private State(float hue, boolean cycle) {
            this.startingHue = hue;
        }

        private State(float hue, double transTime, boolean cycle) {
            this.startingHue = hue;
            this.transitionTime = transTime;
        }

        private State(List<List<Double>> colors, double cycleTime, boolean isCycleColors, double transitionTime) {
            this.colors = colors;
            this.cycleTime = cycleTime;
            this.isCycleColors = isCycleColors;
            this.transitionTime = transitionTime;
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

    public double stateHue = State.RAINBOW.startingHue;
    public float saturation = 1.0f; // Ensures that the colors are on the outside of the color wheel
    public float value = 1.0f; // Hardcoded brightness
    public double startingTransTime = 0.0;
    public double breathingHue = State.BREATHING_PINK.startingHue;

    @Override
    public void writePeriodicOutputs(){
        double timestamp = Timer.getFPGATimestamp();
        if (currentState == State.RAINBOW && currentState.isCycleColors == true) {
            stateHue += 1;
            if (stateHue >= (360 - State.RAINBOW.startingHue)) {
                stateHue = State.RAINBOW.startingHue;
            }

            float rgb[] = new float[3];
            MovingAverage averageR = new MovingAverage(10);
            MovingAverage averageG = new MovingAverage(10);
            MovingAverage averageB = new MovingAverage(10);

            if (saturation > 1) {
                saturation = 1;
            }
            if (saturation < 0) {
                saturation = 0;
            }
            if (value > 1) {
                value = 1;
            }
            if (value < 0) {
                value = 0;
            }
            
            rgb = HSVtoRGB.convert(stateHue, saturation, value);

            rgb[0] = averageR.process(rgb[0]);
            rgb[1] = averageG.process(rgb[1]);
            rgb[2] = averageB.process(rgb[2]);

            canifier.setLEDOutput(rgb[0], LEDChannel.LEDChannelA);
            canifier.setLEDOutput(rgb[1], LEDChannel.LEDChannelB);
            canifier.setLEDOutput(rgb[2], LEDChannel.LEDChannelC);

        } else if(!lit && (timestamp - lastOffTime) >= currentState.offTime && currentState.isCycleColors == false){
            setLEDs(currentState.red, currentState.green, currentState.blue);
            lastOnTime = timestamp;
            lit = true;
        } else if(lit && !Double.isInfinite(currentState.onTime) && currentState.isCycleColors == false){
            if((timestamp - lastOnTime) >= currentState.onTime){
                setLEDs(0.0, 0.0, 0.0);
                lastOffTime = timestamp;
                lit = false;
            }
        } else if (currentState == State.BREATHING_PINK && currentState.isCycleColors == true) {
            if (startingTransTime <= currentState.transitionTime) {
                startingTransTime += currentState.transitionTime / 50.0;
                System.out.print("Starting Transition Time: " + startingTransTime);
            } else if (startingTransTime >= currentState.transitionTime) {
                startingTransTime = 0.0;
            }
            float rgb[] = new float[3];
            MovingAverage averageR = new MovingAverage(10);
            MovingAverage averageG = new MovingAverage(10);
            MovingAverage averageB = new MovingAverage(10);

            if (saturation > 1) {
                saturation = 1;
            }
            if (saturation < 0) {
                saturation = 0;
            }

            double valueBasedOnTime = currentState.transitionTime - startingTransTime;

            rgb = HSVtoRGB.convert(State.BREATHING_PINK.startingHue, saturation, valueBasedOnTime);

            rgb[0] = averageR.process(rgb[0]);
            rgb[1] = averageG.process(rgb[1]);
            rgb[2] = averageB.process(rgb[2]);

            canifier.setLEDOutput(rgb[0], LEDChannel.LEDChannelA);
            canifier.setLEDOutput(rgb[1], LEDChannel.LEDChannelB);
            canifier.setLEDOutput(rgb[2], LEDChannel.LEDChannelC);

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

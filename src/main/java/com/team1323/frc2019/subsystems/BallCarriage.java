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
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team254.drivers.LazyTalonSRX;
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

    public BallCarriage(){
        motor = new LazyTalonSRX(Ports.BALL_CARRIAGE);
        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
        setCurrentLimit(20);
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
        SUCKING(0.5),
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
                    if((timestamp - stateEnteredTimestamp) > 1.0){
                        setOpenLoop(0.0);
                    }
                    break;
                default:
                    break;
            }
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

    public Request stateRequest(State newState){
        return new Request(){
        
            @Override
            public void act() {
                conformToState(newState);    
            }

        };
    }

    @Override
    public void outputTelemetry() {
        if(Constants.kDebuggingOutput){
            SmartDashboard.putNumber("Ball Carriage Voltage", motor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Ball Carriage Current", motor.getOutputCurrent());
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }
}

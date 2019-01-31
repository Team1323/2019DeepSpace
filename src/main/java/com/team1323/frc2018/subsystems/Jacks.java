/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.team1323.frc2018.Constants;
import com.team1323.frc2018.Ports;
import com.team254.drivers.LazyTalonSRX;

public class Jacks extends Subsystem {
    private Jacks instance = null;
    public Jacks getInstance(){
        if(instance == null)
            instance = new Jacks();
        return instance;
    }

    LazyTalonSRX motor;

    public Jacks(){
        motor = new LazyTalonSRX(Ports.JACKS);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        motor.setInverted(false);
        motor.setSensorPhase(false);

        motor.config_kP(0, 0.0);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.0);
        motor.config_kF(0, 1023.0 / Constants.kJackMaxSpeed);
        motor.configMotionCruiseVelocity((int)(Constants.kJackMaxSpeed * 1.0));
        motor.configMotionAcceleration((int)(Constants.kJackMaxSpeed * 1.0));
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.auto.actions;

import com.team1323.frc2019.subsystems.Pigeon;

import edu.wpi.first.wpilibj.Timer;

public class WaitToLeaveRampAction implements Action{
    Pigeon pigeon;
    boolean startedDescent = false;
    double startingRoll = 0.0;
    double timeout = 1.0;
    boolean timedOut = false;
    public boolean timedOut(){ return timedOut; }
    double startTime = 0.0;

    final double kAngleTolerance = 1.5;
    final double kMinExitAngle = 5.0;

    public WaitToLeaveRampAction(double timeout){
        pigeon = Pigeon.getInstance();
        this.timeout = timeout;
    }

    @Override
    public boolean isFinished() {
        double rollAngle = pigeon.getRoll();
        //System.out.println("Pigeon roll: " + rollAngle);
        if(Math.abs(rollAngle) >= (startingRoll + kMinExitAngle) && !startedDescent)
            startedDescent = true;
        if(startedDescent && Math.abs(rollAngle - startingRoll) <= kAngleTolerance){
            timedOut = false;
            return true;
        }

        if((Timer.getFPGATimestamp() - startTime) > timeout){
            timedOut = true;
            System.out.println("WaitToLeaveRampAction timed out");
            return true;
        }

        return false;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        startingRoll = pigeon.getRoll();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }
}

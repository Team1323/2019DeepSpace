/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.auto.actions;

import com.team1323.frc2019.subsystems.Probe;

/**
 * 
 */
public class WaitForDiskAction implements Action{
    Probe probe;

    public WaitForDiskAction(){
        probe = Probe.getInstance();
    }

    @Override
    public boolean isFinished() {
        return probe.hasDisk();
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }
}

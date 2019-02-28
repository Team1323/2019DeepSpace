/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.auto.actions;

import com.team1323.frc2019.subsystems.DiskScorer;
import edu.wpi.first.wpilibj.Timer;

/**
 * 
 */
public class WaitForDiskAction implements Action{
    DiskScorer diskScorer;
    double timeout = 15.0;
    double startTime = 0.0;

    public WaitForDiskAction(){
        diskScorer = DiskScorer.getInstance();
    }

    public WaitForDiskAction(double timeout){
        diskScorer = DiskScorer.getInstance();
        this.timeout = timeout;
    }

    @Override
    public boolean isFinished() {
        if(diskScorer.hasDisk()){
            System.out.println("Disk detected");
            return true;
        }else if((Timer.getFPGATimestamp() - startTime) > timeout){
            System.out.println("Disk waiting action timed out");
            return true;
        }
        return false;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }
}

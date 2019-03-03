/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.auto.actions;
import com.team1323.frc2019.RobotState;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class WaitForVisionAction implements Action {
    RobotState robotState;
    double timeout;
    double startTime = 0;

    public WaitForVisionAction(double timeout){
        robotState = RobotState.getInstance();
        this.timeout = timeout;
    }

    @Override
    public boolean isFinished() {
        return robotState.seesTarget() || (Timer.getFPGATimestamp() - startTime) > timeout;
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

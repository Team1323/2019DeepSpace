/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.auto.actions;

import com.team1323.frc2019.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import java.util.List;

/**
 * 
 */
public class WaitForVisionPositionAction implements Action{
    RobotState robotState;
    double x;

    public WaitForVisionPositionAction(double x){
        this.x = x;
        robotState = RobotState.getInstance();
    }

    @Override
    public boolean isFinished() {
        List<Pose2d> targets = robotState.getCaptureTimeFieldToGoal();
        if(targets.size() >= 3){
            Translation2d targetPosition = targets.get(2).getTranslation();
            System.out.println("Current Target X: " + targetPosition.x());
            return targetPosition.x() >= x;
        }
        return false;
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

package com.team1323.frc2019.auto.actions;

import com.team1323.frc2019.subsystems.Swerve;

/**
 * An action designed to wait until there is only a certain (specified) 
 * amount of time left before the completion of a trajectory.
 */
public class RemainingProgressAction implements Action{
    Swerve swerve;
    double targetProgress = 0.0;

    public RemainingProgressAction(double targetProgress){
        swerve = Swerve.getInstance();
        this.targetProgress = targetProgress;
    }

    @Override
    public boolean isFinished() {
        return swerve.getRemainingProgress() <= targetProgress;
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

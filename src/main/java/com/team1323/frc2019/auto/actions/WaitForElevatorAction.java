package com.team1323.frc2019.auto.actions;

import com.team1323.frc2019.subsystems.Superstructure;

public class WaitForElevatorAction implements Action{
	private Superstructure superstructure;
	
	public WaitForElevatorAction(){
		superstructure = Superstructure.getInstance();
	}

	@Override
	public boolean isFinished() {
		return superstructure.requestsCompleted();
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

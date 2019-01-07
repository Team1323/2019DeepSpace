package com.team1323.frc2018.auto.actions;

import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Swerve;

import edu.wpi.first.wpilibj.Timer;

public class WaitForWallAction implements Action{
	double timeout;
	double startTime;
	Swerve swerve;
	Intake intake;

	public WaitForWallAction(double timeout){
		swerve = Swerve.getInstance();
		intake = Intake.getInstance();
		this.timeout = timeout;
	}
	
	@Override
	public boolean isFinished() {
		if(swerve.hasFinishedPath())
			System.out.println("Path finished");
		else if(intake.getHigherCurrent() > 50.0)
			System.out.println("Intake Current high");
		else if((Timer.getFPGATimestamp() - startTime) > timeout)
			System.out.println("Wait for wall timed out");
		return swerve.hasFinishedPath() || (intake.getHigherCurrent() > 50.0) || ((Timer.getFPGATimestamp() - startTime) > timeout);
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

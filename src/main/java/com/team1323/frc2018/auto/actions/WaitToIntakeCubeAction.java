package com.team1323.frc2018.auto.actions;

import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Intake.IntakeState;
import com.team1323.frc2018.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;

public class WaitToIntakeCubeAction implements Action{
	Intake intake;
	Superstructure s;
	double startTime;
	double timeout;
	
	public WaitToIntakeCubeAction(double timeout){
		intake = Intake.getInstance();
		s = Superstructure.getInstance();
		this.timeout = timeout;
	}

	@Override
	public boolean isFinished() {
		if(intake.hasCube()){
			System.out.println("Intake recognizes cube");
			return true;
		}else if((Timer.getFPGATimestamp() - startTime) > timeout){
			System.out.println("Wait for intake timed out");
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		//TODO Turn on the intake or nah?
		startTime = Timer.getFPGATimestamp();
		s.queue(intake.stateRequest(IntakeState.INTAKING));
	}

	@Override
	public void update() {
		
	}

	@Override
	public void done() {
		s.request(intake.stateRequest(IntakeState.CLAMPING));
	}

}

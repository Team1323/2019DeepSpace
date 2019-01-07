package com.team1323.frc2018.auto.actions;

import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Superstructure;
import com.team1323.frc2018.subsystems.Intake.IntakeState;

public class EjectCubeAction extends RunOnceAction{

	@Override
	public void runOnce() {
		Superstructure.getInstance().addForemostActiveRequest(Intake.getInstance().stateRequest(IntakeState.OPEN));
	}

}

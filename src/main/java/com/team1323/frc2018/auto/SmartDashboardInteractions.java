package com.team1323.frc2018.auto;

import com.team1323.frc2018.auto.modes.StandStillMode;
import com.team1323.frc2018.auto.modes.TwoCloseOneBallMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.TWO_CLOSE_ONE_BALL;
    
    private SendableChooser<AutoOption> modeChooser;
    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
    	modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
    	
    	SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    }
    
    public AutoModeBase getSelectedAutoMode(){
        AutoOption selectedOption = (AutoOption) modeChooser.getSelected();
                
        return createAutoMode(selectedOption);
    }
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }
    
    enum AutoOption{
		TWO_CLOSE_ONE_BALL("2 Close, 1 Ball");
    	
    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }
    
    private AutoModeBase createAutoMode(AutoOption option){
    	switch(option){
			case TWO_CLOSE_ONE_BALL:
				return new TwoCloseOneBallMode();
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}

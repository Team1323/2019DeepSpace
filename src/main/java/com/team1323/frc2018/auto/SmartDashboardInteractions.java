package com.team1323.frc2018.auto;

import com.team1323.frc2018.auto.modes.StandStillMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
	private static final String AUTO_OPTIONS = "auto_options";
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.SCALE_ONLY;
    
    private SendableChooser<AutoOption> modeChooser;
    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
    	modeChooser.addDefault(DEFAULT_MODE.name, DEFAULT_MODE);
    	modeChooser.addObject("Switch Only", AutoOption.SWITCH_ONLY);
		//modeChooser.addObject("Scale Only", AutoOption.SCALE_ONLY);
		modeChooser.addObject("2 Switch + 1 Scale", AutoOption.ASSIST);
		//modeChooser.addObject("Four Cube Switch (Test)", AutoOption.FOUR_CUBE_SWITCH);
    	
    	SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    }
    
    public AutoModeBase getSelectedAutoMode(String gameData){
        AutoOption selectedOption =  (AutoOption)  modeChooser.getSelected();
                
        return createAutoMode(selectedOption, gameData);
    }
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }
    
    enum AutoOption{
		SWITCH_ONLY("Switch Only"),
		FOUR_CUBE_SWITCH("Four Cube Switch"),
		SCALE_ONLY("Scale Only"),
		ASSIST("Assist");
    	
    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }
    
    private AutoModeBase createAutoMode(AutoOption option, String gameData){
    	switch(option){
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}

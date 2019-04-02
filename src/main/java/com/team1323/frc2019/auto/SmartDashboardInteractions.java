package com.team1323.frc2019.auto;

import com.team1323.frc2019.auto.modes.CloseFarBallMode;
import com.team1323.frc2019.auto.modes.CloseMidShipMode;
import com.team1323.frc2019.auto.modes.MidCloseShipMode;
import com.team1323.frc2019.auto.modes.StandStillMode;
import com.team1323.frc2019.auto.modes.TwoCloseOneBallMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.TWO_CLOSE_ONE_BALL;

    public static final Alliance STANDARD_CARPET_SIDE = Alliance.BLUE;
    public static final Alliance NONSTANDARD_CARPET_SIDE = Alliance.RED;
    
    private SendableChooser<AutoOption> modeChooser;
    private SendableChooser<Side> sideChooser;
    private SendableChooser<Alliance> allianceChooser;

    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
        modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
        modeChooser.addOption(AutoOption.CLOSE_FAR_BALL.name, AutoOption.CLOSE_FAR_BALL);
        //modeChooser.addOption(AutoOption.MID_CLOSE_SHIP.name, AutoOption.MID_CLOSE_SHIP);
        modeChooser.addOption(AutoOption.CLOSE_MID_SHIP.name, AutoOption.CLOSE_MID_SHIP);

        sideChooser = new SendableChooser<Side>();
        sideChooser.setDefaultOption("Right", Side.RIGHT);
        sideChooser.addOption("Left", Side.LEFT);

        allianceChooser = new SendableChooser<Alliance>();
        allianceChooser.setDefaultOption("Blue", Alliance.BLUE);
        allianceChooser.addOption("Red", Alliance.RED);
    	
        SmartDashboard.putData("Mode Chooser", modeChooser);
        SmartDashboard.putData("Side Chooser", sideChooser);
        SmartDashboard.putData("Alliance Chooser", allianceChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    }
    
    public AutoModeBase getSelectedAutoMode(){
        AutoOption selectedOption = (AutoOption) modeChooser.getSelected();
        Side selectedSide = (Side) sideChooser.getSelected();
        boolean left = (selectedSide == Side.LEFT);
                
        return createAutoMode(selectedOption, left);
    }
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }

    public Alliance getSelectedAlliance(){
        Alliance alliance = (Alliance) allianceChooser.getSelected();
        return alliance;
    }
    
    enum AutoOption{
        TWO_CLOSE_ONE_BALL("2 Close, 1 Ball"),
        CLOSE_FAR_BALL("1 Close, 1 Far, 1 Ball"),
        MID_CLOSE_SHIP("Mid and Close Cargo Ship"),
        CLOSE_MID_SHIP("Close and Mid Cargo Ship");
    	
    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }

    enum Side{
        LEFT, RIGHT
    }

    public enum Alliance{
        RED, BLUE
    }
    
    private AutoModeBase createAutoMode(AutoOption option, boolean left){
    	switch(option){
			case TWO_CLOSE_ONE_BALL:
                return new TwoCloseOneBallMode(left);
            case CLOSE_FAR_BALL:
                return new CloseFarBallMode(left);
            case MID_CLOSE_SHIP:
                return new MidCloseShipMode(left);
            case CLOSE_MID_SHIP:
                return new CloseMidShipMode(left);
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}

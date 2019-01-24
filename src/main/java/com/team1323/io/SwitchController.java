package com.team1323.io;

import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;

public class SwitchController extends GenericHID{
    private double DEAD_BAND = 0.15;
    private boolean rumbling = false;
    public ButtonCheck aButton, bButton, xButton, yButton, plusButton, minusButton,
    	leftBumper, rightBumper, leftCenterClick, rightCenterClick, leftTrigger, 
    	rightTrigger, POV0, POV90, POV180, POV270, homeButton, captureButton;
    public static final int A_BUTTON = 2;
    public static final int B_BUTTON = 1;
    public static final int X_BUTTON = 4;
    public static final int Y_BUTTON = 3;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int MINUS_BUTTON = 9;
    public static final int PLUS_BUTTON = 10;
    public static final int LEFT_CENTER_CLICK = 11;
    public static final int RIGHT_CENTER_CLICK = 12;
    public static final int LEFT_TRIGGER = 7;
    public static final int RIGHT_TRIGGER = 8;
    public static final int CAPTURE_BUTTON = 14;
    public static final int HOME_BUTTON = 13;
    public static final int POV_0 = -4;
    public static final int POV_90 = -5;
    public static final int POV_180 = -6;
    public static final int POV_270 = -7;
    
    public void setDeadband(double deadband){
    	DEAD_BAND = deadband;
    }
    
    public SwitchController(int usb)   { 
    	super(usb);
    	aButton = new ButtonCheck(A_BUTTON);
        bButton = new ButtonCheck(B_BUTTON);
        xButton = new ButtonCheck(X_BUTTON);
        yButton = new ButtonCheck(Y_BUTTON);
        minusButton = new ButtonCheck(MINUS_BUTTON);
        plusButton = new ButtonCheck(PLUS_BUTTON);
        captureButton = new ButtonCheck(CAPTURE_BUTTON);
        homeButton = new ButtonCheck(HOME_BUTTON);
        leftBumper = new ButtonCheck(LEFT_BUMPER);
        rightBumper = new ButtonCheck(RIGHT_BUMPER);
        leftCenterClick = new ButtonCheck(LEFT_CENTER_CLICK);
        rightCenterClick = new ButtonCheck(RIGHT_CENTER_CLICK);     
        leftTrigger = new ButtonCheck(LEFT_TRIGGER);
        rightTrigger = new ButtonCheck(RIGHT_TRIGGER);
        POV0 = new ButtonCheck(POV_0);
        POV90 = new ButtonCheck(POV_90);
        POV180 = new ButtonCheck(POV_180);
        POV270 = new ButtonCheck(POV_270);
   }
    
    @Override
    public double getX(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
          return Util.deadBand(getRawAxis(0), DEAD_BAND);
        } else {
          return Util.deadBand(getRawAxis(2), DEAD_BAND);
        }
      }
    @Override
    public double getY(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
          return Util.deadBand(getRawAxis(1), DEAD_BAND);
        } else {
          return Util.deadBand(getRawAxis(3), DEAD_BAND);
        }
      }
    
    public void rumble(double rumblesPerSecond, double numberOfSeconds){
    	if(!rumbling){
    		RumbleThread r = new RumbleThread(rumblesPerSecond, numberOfSeconds);
    		r.start();
    	}
    }
    public boolean isRumbling(){
    	return rumbling;
    }
    public class RumbleThread extends Thread{
    	public double rumblesPerSec = 1;
    	public long interval = 500;
    	public double seconds = 1;
    	public double startTime = 0;
    	public RumbleThread(double rumblesPerSecond, double numberOfSeconds){
    		rumblesPerSec = rumblesPerSecond;
    		seconds = numberOfSeconds;
    		interval =(long) (1/(rumblesPerSec*2)*1000);
    	}
    	public void run(){
    		rumbling = true;
    		startTime = Timer.getFPGATimestamp();
    		try{
    			while((Timer.getFPGATimestamp() - startTime) < seconds){
		    		setRumble(RumbleType.kLeftRumble, 1);
		    		setRumble(RumbleType.kRightRumble, 1);
		    		sleep(interval);
		    		setRumble(RumbleType.kLeftRumble, 0);
		    		setRumble(RumbleType.kRightRumble, 0);
		    		sleep(interval);
    			}
    		}catch (InterruptedException e) {
				rumbling = false;
				e.printStackTrace();
			}
    		rumbling = false;
    	}
    }
    
    public class ButtonCheck{
    	boolean buttonCheck = false;
    	boolean buttonActive = false;
    	boolean longPressed = false;
    	boolean longPressActivated = false;
    	boolean hasBeenPressed = false;
    	boolean longReleased = false;
    	private double buttonStartTime = 0;
    	private int buttonNumber;
    	
    	public ButtonCheck(int id){
    		buttonNumber = id;
    	}
    	public void update(){
    		if(buttonNumber > 0){
    			buttonCheck = getRawButton(buttonNumber);
    		}else{
    			switch(buttonNumber){
    				case POV_0:
    					buttonCheck = (getPOV() == 0);
    					break;
    				case POV_90:
    					buttonCheck = (getPOV() == 90);
    					break;
    				case POV_180:
    					buttonCheck = (getPOV() == 180);
    					break;
    				case POV_270:
    					buttonCheck = (getPOV() == 270);
    					break;
    				default:
    					buttonCheck = false;
    					break;
    			}
    		}
    		if(buttonCheck){
	    		if(buttonActive){
	    			if((System.currentTimeMillis() - buttonStartTime > 250) && !longPressActivated){
	    				longPressActivated = true;
	    				longPressed = true;
	    			}
	    		}else{
	    			buttonActive = true;
	    			buttonStartTime = System.currentTimeMillis();
	    		}
    		}else{
    			if(buttonActive){
    				buttonActive = false;
    				if(longPressActivated){
    					hasBeenPressed = false;
    					longPressActivated = false;
    					longPressed = false;
    					longReleased = true;
    				}else{
    					hasBeenPressed = true;
    				}
    			}
    		}
    	}
    	public boolean wasPressed(){
    		if(hasBeenPressed){
    			hasBeenPressed = false;
    			return true;
    		}
    		return false;
    	}
    	public boolean longPressed(){
    		if(longPressed){
    			longPressed = false;
    			return true;
    		}
    		return false;
    	}
    	public boolean longReleased(){
    		if(longReleased){
    			longReleased = false;
    			return true;
    		}
    		return false;
    	}
    	public boolean isBeingPressed(){
    		return buttonActive;
    	}
    }
    public void update(){
    	aButton.update();
    	bButton.update();
    	xButton.update();
    	yButton.update();
        minusButton.update();
        plusButton.update();
        captureButton.update();
        homeButton.update();
    	leftBumper.update();
    	rightBumper.update();
    	leftCenterClick.update();
    	rightCenterClick.update();
    	leftTrigger.update();
    	rightTrigger.update();
    	POV0.update();
    	POV90.update();
    	POV180.update();
    	POV270.update();
    }
}
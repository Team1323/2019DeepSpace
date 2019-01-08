package com.team1323.lib.util;


public class InputRamp{
    double output = 0.0;
    double lastInput = 0.0;
    double rampRate;
    double center;
    double lastTimestamp = 0.0;
    double deadband;
    boolean ramping = false;
    double rampDirection;

    /**
     * 
     * @param center output is only ramped as it approaches the center
     * @param rate time from output 0 to output 1, in seconds
     * @param deadband ramping stops when the output is within one deadband of the center
     */
    public InputRamp(double center, double rate, double deadband){
        this.center = center;
        rampRate = rate;
        this.deadband = deadband;
    }

    /**
     * 
     * @param rate time from output 0 to output 1, in seconds
     */
    public void setRampRate(double rate){
        rampRate = rate;
    }

    public double update(double input, double timestamp){
        double distance = Math.abs(input - center);
        double lastDistance = Math.abs(lastInput - center);
        if((input == center || distance < lastDistance) && output != input && Math.abs(input - output) > deadband){
            double direction = Math.signum(input - output);
            if(!ramping){
                rampDirection = direction;
                ramping = true;
            }else if(direction != rampDirection){
                return input;
            }
            output += (direction * ((timestamp - lastTimestamp) / rampRate));
        }else{
            ramping = false;
            output = input;
        }

        lastTimestamp = timestamp;
        lastInput = input;

        return output;
    }

    public double getOutput(){
        return output;
    }

    public void reset(double timestamp){
        output = 0.0;
        lastInput = 0.0;
        lastTimestamp = timestamp;
        ramping = false;
    }
}
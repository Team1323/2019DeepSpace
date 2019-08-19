package com.team254.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1323.frc2019.Constants;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonSRX extends TalonSRX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;
    
    boolean kSimulated = Constants.kSimulate;

    //Simulation variables
    double simPercentOutput = 0.0;
    double simVoltage = 0.0;
    double simVoltageCompSat = 12.0;
    double simCurrent = 0.0;
    double simRampRate = 0.0;
    int simSensorPosition = 0;

    public LazyTalonSRX(int deviceNumber) {
        super(deviceNumber);
        if(Constants.kResetTalons) super.configFactoryDefault();
    }

    public void simulate(boolean simulate){
        kSimulated = simulate;
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;

            if(!kSimulated){
                super.set(mode, value);
            }
        }
    }

    @Override
    public double getMotorOutputVoltage(){
        if(kSimulated) return simVoltage;
        return super.getMotorOutputVoltage();
    }

    @Override
    public double getOutputCurrent(){
        if(kSimulated) return simCurrent;
        return super.getOutputCurrent();
    }

    @Override
    public int getSelectedSensorPosition(int pidIdx){
        if(kSimulated) return simSensorPosition;
        return super.getSelectedSensorPosition(pidIdx);
    }
}
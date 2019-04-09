/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.team1323.frc2019.Constants;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team254.drivers.LazyTalonSRX;
import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Jacks extends Subsystem {
    private static Jacks instance = null;
    public static Jacks getInstance(){
        if(instance == null)
            instance = new Jacks();
        return instance;
    }

    LazyTalonSRX motor;

    Solenoid PTOShifter;

    DiskIntake diskIntake;

    PeriodicIO periodicIO = new PeriodicIO();

    double targetHeight = 0.0;

    public Jacks(){
        diskIntake = DiskIntake.getInstance();

        motor = diskIntake.getTalon();
        motor.setInverted(false);

        if(!Constants.kIsUsingCompBot){
            motor.configRemoteFeedbackFilter(Ports.DISK_SCORER, RemoteSensorSource.TalonSRX_SelectedSensor, 0);
            motor.configRemoteFeedbackFilter(Ports.DISK_SCORER, RemoteSensorSource.Off, 1);
            motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
            motor.setSensorPhase(false);
        }else{
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            motor.setSensorPhase(true);
        }

        resetToAbsolutePosition();

        PTOShifter = new Solenoid(Ports.DRIVEBASE_PCM, Ports.PTO_SHIFTER);

        shiftPower(false);
    }

    private void configureTalon(){
        motor.config_kP(0, 0.5);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 5.0);
        motor.config_kF(0, 1023.0 / Constants.kJackMaxSpeed);
        motor.configMotionCruiseVelocity((int)(Constants.kJackMaxSpeed * 1.0));
        motor.configMotionAcceleration((int)(Constants.kJackMaxSpeed * 3.0));

        motor.configForwardSoftLimitThreshold((int)(jackHeightToEncUnits(Constants.kJackMaxControlHeight)));
        motor.configReverseSoftLimitThreshold((int)(jackHeightToEncUnits(Constants.kJackMinControlHeight)));
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);

        motor.configContinuousCurrentLimit(Constants.kJackCurrentLimit);
        motor.configPeakCurrentLimit(Constants.kJackCurrentLimit);
        motor.configPeakCurrentDuration(10);
        motor.enableCurrentLimit(true);

        setOpenLoop(0.0);
    }

    public enum ControlState{
        OPEN_LOOP, POSITION
    }
    ControlState state = ControlState.OPEN_LOOP;
    public ControlState getState(){ return state; }

    boolean hasPower = false;
    public void shiftPower(boolean shiftToJacks){
        hasPower = shiftToJacks;
        diskIntake.shiftPower(shiftToJacks);
        PTOShifter.set(shiftToJacks);
        if(shiftToJacks)
            configureTalon();
    }

    public boolean isOpenLoop(){
        return state == ControlState.OPEN_LOOP;
    }

    public synchronized void setOpenLoop(double percentOutput){
        state = ControlState.OPEN_LOOP;
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.setpoint = percentOutput;
    }

    public synchronized void setHeight(double height){
        if(hasPower){
            if(height > Constants.kJackMaxControlHeight)
                height = Constants.kJackMaxControlHeight;
            else if(height < Constants.kJackMinControlHeight)
                height = Constants.kJackMinControlHeight;

            if(isSensorConnected()){
                state = ControlState.POSITION;
                periodicIO.controlMode = ControlMode.MotionMagic;
                periodicIO.setpoint = jackHeightToEncUnits(height);
                targetHeight = height;
                System.out.println("Jack height set to: " + height);
            }else{
                DriverStation.reportError("Jack encoder not detected!", false);
                setOpenLoop(0.0);
            }
        }else{
            DriverStation.reportError("Illegal jack height request", false);
        }
    }

    public synchronized void lockHeight(){
        setHeight(getHeight());
    }

    public Request openLoopRequest(double input){
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(input);
            }
        };
    }

    public Request heightRequest(double height){
        return new Request(){
        
            @Override
            public void act() {
                setHeight(height);    
            }

            @Override
            public boolean isFinished(){
                return hasReachedTargetHeight() || isOpenLoop();
            }

        };
    }

    public Request lockHeightRequest(){
        return new Request(){
        
            @Override
            public void act() {
                lockHeight();
            }
        };
    }

    public Request shiftPowerRequest(boolean shiftToJacks){
        return new Request(){
        
          @Override
          public void act() {
            shiftPower(shiftToJacks);
          }
    
        };
      }

    public double getHeight(){
        return encUnitsToJackHeight(periodicIO.position);
    }

    public boolean hasReachedTargetHeight(){
        return Math.abs(targetHeight - getHeight()) <= Constants.kJackHeightTolerance;
    }

    public boolean isAtHeight(double height){
        return Math.abs(height - getHeight()) <= Constants.kJackHeightTolerance;
    }

    private double encUnitsToInches(double encUnits){
        return encUnits / Constants.kJackTicksPerInch;
    }

    private double inchesToEncUnits(double inches){
        return inches * Constants.kJackTicksPerInch;
    }

    private double encUnitsToJackHeight(double encUnits){
        return encUnitsToInches(encUnits - Constants.kJackStartingEncPosition);
    }

    private double jackHeightToEncUnits(double jackHeight){
        return Constants.kJackStartingEncPosition + inchesToEncUnits(jackHeight);
    }

    public boolean isSensorConnected(){
        if(!Constants.kIsUsingCompBot){
            return DiskScorer.getInstance().isSensorConnected();
        }else{
            int pulseWidthPeriod = motor.getSensorCollection().getPulseWidthRiseToRiseUs();
            boolean connected = pulseWidthPeriod != 0;
            if(!connected)
                hasEmergency = true;
            return connected;
        }
	}

    public void resetToAbsolutePosition(){
        if(!Constants.kIsUsingCompBot){
            int absolutePosition = (int) Util.boundToScope(0, 4096, DiskScorer.getInstance().getTalon().getSensorCollection().getPulseWidthPosition());
            System.out.println("Pulse width position: " + DiskScorer.getInstance().getTalon().getSensorCollection().getPulseWidthPosition());
            if(encUnitsToJackHeight(absolutePosition) > Constants.kJackMaxPhysicalHeight){
                absolutePosition -= 4096;
            }else if(encUnitsToJackHeight(absolutePosition) < Constants.kJackMinPhysicalHeight){
                absolutePosition += 4096;
            }
            double jackHeight = encUnitsToJackHeight(absolutePosition);
            if(jackHeight > Constants.kJackMaxPhysicalHeight || jackHeight < Constants.kJackMinPhysicalHeight){
                DriverStation.reportError("Jack height is out of bounds", false);
                hasEmergency = true;
            }
            DiskScorer.getInstance().setSensorPosition(absolutePosition);
        }else{
            int absolutePosition = (int) Util.boundToScope(0, 4096, motor.getSensorCollection().getPulseWidthPosition());
            System.out.println("Pulse width position: " + motor.getSensorCollection().getPulseWidthPosition());
            if(encUnitsToJackHeight(absolutePosition) > Constants.kJackMaxPhysicalHeight){
                absolutePosition -= 4096;
            }else if(encUnitsToJackHeight(absolutePosition) < Constants.kJackMinPhysicalHeight){
                absolutePosition += 4096;
            }
            double jackHeight = encUnitsToJackHeight(absolutePosition);
            if(jackHeight > Constants.kJackMaxPhysicalHeight || jackHeight < Constants.kJackMinPhysicalHeight){
                DriverStation.reportError("Jack height is out of bounds", false);
                hasEmergency = true;
            }
            motor.setSelectedSensorPosition(absolutePosition);
        }
	}
    
    @Override
    public void readPeriodicInputs() {
        periodicIO.position = motor.getSelectedSensorPosition();
        if(Constants.kDebuggingOutput){
            periodicIO.velocity = motor.getSelectedSensorVelocity();
            periodicIO.voltage = motor.getMotorOutputVoltage();
            periodicIO.current = motor.getOutputCurrent();
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if(hasPower){
            motor.set(periodicIO.controlMode, periodicIO.setpoint);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Jack Height", getHeight());
        SmartDashboard.putBoolean("Jacks Have Power", hasPower);
        if(Constants.kDebuggingOutput){
            SmartDashboard.putNumber("Jack Velocity", periodicIO.velocity);
            SmartDashboard.putNumber("Jack Voltage", periodicIO.voltage);
            SmartDashboard.putNumber("Jack Current", periodicIO.current);
            SmartDashboard.putNumber("Jack Encoder", periodicIO.position);
            SmartDashboard.putNumber("Jack Pulse Width", motor.getSensorCollection().getPulseWidthPosition());
            SmartDashboard.putNumber("Jack Error", encUnitsToInches(motor.getClosedLoopError()));
        }
    }

    @Override
    public void stop() {
        //shiftPower(false);
    }

    public static class PeriodicIO{
        //inputs
        public int position = 0;
        public int velocity = 0;
        public double voltage = 0.0;
        public double current = 0.0;

        //outputs
        public double setpoint = 0.0;
        public ControlMode controlMode = ControlMode.PercentOutput;
    }
}

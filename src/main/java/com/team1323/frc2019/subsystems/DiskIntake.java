/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.team1323.frc2019.Ports;
import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.subsystems.Subsystem;
import com.team1323.frc2019.subsystems.requests.Prerequisite;
import com.team1323.frc2019.subsystems.requests.Request;
import com.team254.drivers.LazyTalonSRX;
import com.team1323.frc2019.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages the ground disk intake
 **/
public class DiskIntake extends Subsystem {
  private static DiskIntake instance = null;
  public static DiskIntake getInstance() {
    if(instance == null) 
      instance = new DiskIntake();
    return instance;
  }

  private boolean hasDisk = false;
  public synchronized boolean hasDisk() {
    return hasDisk;
  }
  public void feignDisk(){
    hasDisk = false;
  }

  private LazyTalonSRX diskMotor;
  public LazyTalonSRX getTalon(){
    return diskMotor;
  }

  private Solenoid lift;

  private DiskIntake() {
    diskMotor = new LazyTalonSRX(Ports.DISK_INTAKE);
    lift = new Solenoid(Ports.DRIVEBASE_PCM, Ports.DISK_INTAKE_LIFT);

    diskMotor.setInverted(false);

    diskMotor.setNeutralMode(NeutralMode.Brake);

    diskMotor.configVoltageCompSaturation(12.0, 10);
    diskMotor.enableVoltageCompensation(true);
  }

  private void configureTalon(){
    diskMotor.configForwardSoftLimitEnable(false);
    diskMotor.configReverseSoftLimitEnable(false);

    setCurrentLimit(Constants.kJackCurrentLimit);
  }

  public void setCurrentLimit(int amps) {
    diskMotor.configContinuousCurrentLimit(amps, 10);
    diskMotor.configPeakCurrentLimit(amps, 10);
    diskMotor.configPeakCurrentDuration(10, 10);
    diskMotor.enableCurrentLimit(true);
  }

  public void enableCurrentLimit(boolean enable) {
    diskMotor.enableCurrentLimit(enable);
  }

  private void setRampRate(double secondsToMax) {
    diskMotor.configOpenloopRamp(secondsToMax, 0);
  }

  public enum State {
    OFF(0, true), 
    INTAKING(Constants.kDiskIntakingOutput, false), 
    EJECTING(Constants.kDiskIntakeStrongEjectOutput, false), 
    HANDOFF_COMPLETE(0, true), 
    HOLDING(Constants.kDiskStrongHoldingOutput, true),
    DEPLOYED(0, false),
    DELIVERING(4.0/12.0, true),
    DISABLED(0.0, true);

    public double diskIntakeOutput = 0;
    public boolean lifted = false;

    private State(double output, boolean lift) {
      diskIntakeOutput = output;
      lifted = lift;
    }

  }

  private State currentState = State.OFF;
  public State getState() {
    return currentState;
  }

  private double stateEnteredTimestamp = 0;
  private boolean stateChanged = false;
  private double currentSpikeTimestamp = Double.POSITIVE_INFINITY;
  private boolean isResucking = false;
  private double holdingOutput = Constants.kDiskIntakeWeakEjectOutput;
  private boolean needsToNotifyDrivers = false;

  private synchronized void setState(State newState) {
    if (newState != currentState) {
      stateChanged = true;
    }
    currentState = newState;
    stateEnteredTimestamp = Timer.getFPGATimestamp();
  }

  public void setHoldingOutput(double output) {
    holdingOutput = output;
  }

  public boolean needsToNotifyDivers() {
    if (needsToNotifyDrivers) {
      needsToNotifyDrivers = false;
      return true;
    }
    return false;
  }

  public void fireLift(boolean fire) {
    lift.set(!fire);
  }

  private void setRollers(double speed) {
    setRampRate(0.0);
    diskMotor.set(ControlMode.PercentOutput, speed);
  }

  private void holdRollers() {
    setRollers(holdingOutput);
  }

  boolean hasPower = true;
  public void shiftPower(boolean shiftToJacks){
    hasPower = !shiftToJacks;
    conformToState(shiftToJacks ? State.DISABLED : State.OFF);
    if(!shiftToJacks)
      configureTalon();
  }

  private final Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
      hasDisk = false;
      needsToNotifyDrivers = false;
      setState(State.OFF);
      stop();
    }

    @Override
    public void onLoop(double timestamp) {

      switch (currentState) {
        case OFF:
          
          break;
        case INTAKING:
          if(stateChanged)
            hasDisk = false;
          if(diskMotor.getOutputCurrent() >= 10.0 && (timestamp - stateEnteredTimestamp) >= 0.5) {
            if(Double.isInfinite(currentSpikeTimestamp)) {
              currentSpikeTimestamp = timestamp;
            } else {
              if(timestamp - currentSpikeTimestamp > 0.375) {
                hasDisk = true;
                needsToNotifyDrivers = true;
              }
            }
          } else if (!Double.isInfinite(currentSpikeTimestamp)) {
            currentSpikeTimestamp = Double.POSITIVE_INFINITY;
          }
          break;
        case EJECTING:
          if(stateChanged) {
            //setRampRate(0.0);
            hasDisk = false;
          }
          if (timestamp - stateEnteredTimestamp > 2.0) {
            stop();
            //setRampRate(Constants.kDiskIntakeRampRate);
          }
          break;
        case HANDOFF_COMPLETE:
          if(stateChanged)
            hasDisk = false;
          break;
        case HOLDING:
          /*if(banner.get()) {
            if(isResucking) {
              holdRollers();
              isResucking = false;
            }
          } else {
            if (!isResucking) {
              setRollers(Constants.kDiskIntakingResuckingOutput);
              isResucking = true;
            }
          }*/
          break;
        default:
          break;
      }

      if(stateChanged)
        stateChanged = false;

    }

    @Override
    public void onStop(double timestamp) {
      setState(State.OFF);
      stop();
    }

  };

  public void eject(double output) {
    setState(State.EJECTING);
    setRollers(output);
    fireLift(false);
    hasDisk = false;
  }

  public void conformToState(State desiredState) {
    conformToState(desiredState, desiredState.diskIntakeOutput);
  }

  public void conformToState(State desiredState, double outputOverride) {
    if(hasPower || (!hasPower && desiredState == State.DISABLED)){
      setState(desiredState);
      setRollers(outputOverride);
      fireLift(desiredState.lifted);
    }else{
      DriverStation.reportError("Disk intake state change not allowed", false);
    }
  }

  public Request stateRequest(State desiredState) {
    return new Request(){
    
      @Override
      public void act() {
        conformToState(desiredState);
      }
    };
  }

  public Request waitForDiskRequest() {
    return new Request(){
    
      @Override
      public void act() {
        conformToState(State.INTAKING);
      }

      @Override
      public boolean isFinished() {
        return !stateChanged && hasDisk;
      }

    };
  }

  public Request ejectRequest(double output) {
    return new Request(){
    
      @Override
      public void act() {
        conformToState(State.EJECTING, output);
      }

    };
  }

  public Prerequisite diskRequisite() {
    return new Prerequisite(){
    
      @Override
      public boolean met() {
        return hasDisk();
      }
    };
  }

  @Override
  public synchronized void stop() {
    conformToState(State.OFF);
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putString("Disk intake state", currentState.toString());
    if(Constants.kDebuggingOutput) {
      SmartDashboard.putNumber("Disk Intake Current", diskMotor.getOutputCurrent());
      SmartDashboard.putNumber("Disk Intake Voltage", diskMotor.getMotorOutputVoltage());
      SmartDashboard.putBoolean("Disk Intake Has Disk", hasDisk);
    }

  }
}

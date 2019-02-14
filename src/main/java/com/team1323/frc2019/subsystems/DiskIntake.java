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

  boolean hasDisk = false;
  public synchronized boolean hasDisk() {
    return hasDisk;
  }

  private LazyTalonSRX diskMotor;
  private Solenoid lift;
  private DigitalInput banner;

  public boolean getBanner() {
    return banner.get();
  }

  private DiskIntake() {
    diskMotor = new LazyTalonSRX(Ports.DISK_INTAKE);
    lift = new Solenoid(Ports.DRIVEBASE_PCM, Ports.DISK_INTAKE_LIFT);
    banner = new DigitalInput(Ports.DISK_INTAKE_BANNER);

    diskMotor.setInverted(false);

    diskMotor.setNeutralMode(NeutralMode.Brake);

    diskMotor.configVoltageCompSaturation(12.0, 10);
    diskMotor.enableVoltageCompensation(true);

    diskMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 10);
    diskMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 10);

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
    OFF(0, false), 
    INTAKING(Constants.kDiskIntakingOutput, false), 
    EJECTING(Constants.kDiskIntakeStrongEjectOutput, true), 
    HANDOFF(Constants.kDiskIntakeWeakEjectOutput, true), 
    HOLDING(Constants.kDiskStrongHoldingOutput, true);

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
  private double bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
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
          if(banner.get()) {
            if(Double.isInfinite(bannerSensorBeganTimestamp)) {
              bannerSensorBeganTimestamp = timestamp;
            } else {
              if(timestamp - bannerSensorBeganTimestamp > 0.1) {
                hasDisk = true;
                needsToNotifyDrivers = true;
              }
            }
          } else if (!Double.isInfinite(bannerSensorBeganTimestamp)) {
            bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
          }
          break;
        case EJECTING:
          if(stateChanged) {
            setRampRate(0.0);
            hasDisk = false;
          }
          if (timestamp - stateEnteredTimestamp > 2.0) {
            stop();
            setRampRate(Constants.kDiskIntakeRampRate);
          }
          break;
        case HANDOFF:
          if(banner.get()) {
            if (timestamp - stateEnteredTimestamp > 2.0) {
              setRampRate(Constants.kDiskIntakeRampRate);
              setRollers(Constants.kDiskIntakeWeakEjectOutput);
            }
          }
          break;
        case HOLDING:
          if(banner.get()) {
            if(isResucking) {
              holdRollers();
              isResucking = false;
            }
          } else {
            if (!isResucking) {
              setRollers(Constants.kDiskIntakingResuckingOutput);
              isResucking = true;
            }
          }
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

  private void conformToState(State desiredState) {
    setState(desiredState);
    setRollers(desiredState.diskIntakeOutput);
    fireLift(desiredState.lifted);
  }

  private void conformToState(State desiredState, double outputOverride) {
    setState(desiredState);
    setRollers(outputOverride);
    fireLift(desiredState.lifted);
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
    if(Constants.kDebuggingOutput) {
      SmartDashboard.putNumber("Disk Intake Current", diskMotor.getOutputCurrent());
      SmartDashboard.putNumber("Disk Intake Voltage", diskMotor.getMotorOutputVoltage());
      SmartDashboard.putBoolean("Disk Intake Has Cube", hasDisk);
      SmartDashboard.putBoolean("Disk Intake Banner", banner.get());
    }

  }
}

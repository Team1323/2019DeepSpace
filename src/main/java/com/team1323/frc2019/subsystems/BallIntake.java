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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallIntake extends Subsystem {
  private static BallIntake instance = null;

  public static BallIntake getInstance() {
    if (instance == null)
      instance = new BallIntake();
    return instance;
  }

  boolean hasBall = false;

  public synchronized boolean hasBall() {
    return hasBall;
  }
  public void feignBall(){
    hasBall = true;
  }

  private LazyTalonSRX grabber, feeder;
  private DigitalInput banner;

  public LazyTalonSRX getPigeonTalon() {
    return feeder;
  }

  public boolean getBanner() {
    return banner.get();
  }

  private BallIntake() {
    grabber = new LazyTalonSRX(Ports.BALL_INTAKE);
    feeder = new LazyTalonSRX(Ports.BALL_FEEDER);
    banner = new DigitalInput(Ports.BALL_INTAKE_BANNER);

    grabber.setInverted(true);

    grabber.setNeutralMode(NeutralMode.Brake);

    grabber.configVoltageCompSaturation(12.0, 10);
    grabber.enableVoltageCompensation(true);

    grabber.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 10);
    grabber.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 10);

    feeder.setInverted(false);

    feeder.setNeutralMode(NeutralMode.Brake);

    feeder.configVoltageCompSaturation(12.0, 10);
    feeder.enableVoltageCompensation(true);

    feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 10);
    feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 10);

    setCurrentLimit(30);
  }

  public void setCurrentLimit(int amps) {
    grabber.configContinuousCurrentLimit(amps, 10);
    grabber.configPeakCurrentLimit(amps);
    grabber.configPeakCurrentDuration(10, 10);
    grabber.enableCurrentLimit(true);

    feeder.configContinuousCurrentLimit(amps, 10);
    feeder.configPeakCurrentLimit(amps);
    feeder.configPeakCurrentDuration(10, 10);
    feeder.enableCurrentLimit(true);
  }

  public void enableCurrrentLimit(boolean enable) {
    grabber.enableCurrentLimit(enable);
    feeder.enableCurrentLimit(enable);
  }

  private void setRampRate(double secondsToMax) {
    grabber.configOpenloopRamp(secondsToMax, 0);
    feeder.configOpenloopRamp(secondsToMax, 0);
  }

  public enum State {
    OFF(0, 0), INTAKING(Constants.kIntakingOutput, 0.5),
    EJECTING(Constants.kIntakeEjectOutput, Constants.kIntakeEjectOutput),
    HOLDING(Constants.kIntakingOutput, 0.5),
    CLIMBING(Constants.kIntakeClimbOutput, 0),
    PULLING(Constants.kIntakePullOutput, 0),
    FEEDING(Constants.kIntakeWeakHoldingOutput, 0.5),
    POST_FEEDING(0, 0);

    public double grabberOutput = 0;
    public double feederOutput = 0;

    private State(double grabberSpeed, double feederSpeed) {
      grabberOutput = grabberSpeed;
      feederOutput = feederSpeed;
    }
  }

  private State currentState = State.OFF;
  private boolean stateChanged = false;
  private double bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
  private double stateEnteredTimestamp = 0;
  private double holdingOutput = Constants.kIntakeWeakHoldingOutput;
  private boolean isConstantSuck = false;

  public State getState() {
    return currentState;
  }

  private synchronized void setState(State newState) {
    if (newState != currentState)
      stateChanged = true;
    currentState = newState;
    stateEnteredTimestamp = Timer.getFPGATimestamp();
  }

  public void setHoldingOutput(double output) {
    holdingOutput = output;
  }

  private boolean needsToNotifyDrivers = false;

  public boolean needsToNotifyDrivers() {
    if (needsToNotifyDrivers) {
      needsToNotifyDrivers = false;
      return true;
    }
    return false;
  }

  private void setGrabberSpeed(double output) {
    setRampRate(0.0);
    grabber.set(ControlMode.PercentOutput, output);
  }

  private void setFeederSpeed(double output) {
    feeder.set(ControlMode.PercentOutput, output);
  }

  private void holdRollers() {
    setGrabberSpeed(holdingOutput);
  }

  private final Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
      hasBall = false;
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
        if (stateChanged)
          hasBall = false;
        if (/*banner.get()*/ (grabber.getOutputCurrent() >= 10.0) && ((timestamp - stateEnteredTimestamp) >= 0.5)) {
          if (Double.isInfinite(bannerSensorBeganTimestamp)) {
            bannerSensorBeganTimestamp = timestamp;
          } else {
            if (timestamp - bannerSensorBeganTimestamp > 0.3) {
              hasBall = true;
              needsToNotifyDrivers = true;
            }
          }
        } else if (!Double.isFinite(bannerSensorBeganTimestamp)) {
          bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
        }
        break;
      case EJECTING:
        if (stateChanged) {
          setRampRate(0.0);
          hasBall = false;
        }
        if (timestamp - stateEnteredTimestamp > 2.0) {
          stop();
          setRampRate(Constants.kIntakeRampRate);
        }
        break;
      case HOLDING:
        /*if (banner.get()) {
          if (isConstantSuck) {
            holdRollers();
            isConstantSuck = false;
          }
        } else {
          if (!isConstantSuck) {
            setGrabberSpeed(Constants.kIntakingResuckingOutput);
            isConstantSuck = true;
          }
        }*/
        if((timestamp - stateEnteredTimestamp) >= 1.0){
          setGrabberSpeed(Constants.kIntakeWeakHoldingOutput);
        }
        break;
      case CLIMBING:
        break;
      case POST_FEEDING:
        if(stateChanged)
          hasBall = false;
        break;
      default:
        break;
      }
      if (stateChanged)
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
    setGrabberSpeed(output);
    hasBall = false;
  }

  public void conformToState(State desiredState) {
    setState(desiredState);
    setGrabberSpeed(desiredState.grabberOutput);
    setFeederSpeed(desiredState.feederOutput);
  }

  public void conformToState(State desiredState, double outputOverride) {
    setState(desiredState);
    setGrabberSpeed(outputOverride);
    setFeederSpeed(outputOverride);
  }

  public Request stateRequest(State desiredState) {
    return new Request() {

      @Override
      public void act() {
        conformToState(desiredState);
      }

    };

  }

  public Request waitForBallRequest() {
    return new Request() {

      @Override
      public void act() {
        conformToState(State.INTAKING);
      }

      @Override
      public boolean isFinished() {
        return !stateChanged && hasBall;
      }

    };
  }

  public Request ejectRequest(double output) {
    return new Request() {

      @Override
      public void act() {
        conformToState(State.EJECTING, output);
      }
    };
  }

  public Prerequisite ballRequisite() {
    return new Prerequisite() {

      @Override
      public boolean met() {
        return hasBall();
      }
    };
  }

  @Override
  public void outputTelemetry() {
    if (Constants.kDebuggingOutput) {
      SmartDashboard.putNumber("Intake Grabber Current", grabber.getOutputCurrent());
      SmartDashboard.putNumber("Intake Grabber Voltage", grabber.getMotorOutputVoltage());
      SmartDashboard.putNumber("Intake Feeder Current", feeder.getOutputCurrent());
      SmartDashboard.putNumber("Intake Feeder Voltage", feeder.getMotorOutputVoltage());
      SmartDashboard.putBoolean("Intake Has Ball", hasBall);
      SmartDashboard.putBoolean("Intake Banner", banner.get());
    }

  }

  @Override
  public synchronized void stop() {
    conformToState(State.OFF);
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

}

package com.team1323.frc2019.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.team1323.frc2019.loops.ILooper;
import com.team1323.frc2019.loops.Loop;
import com.team1323.frc2019.loops.Looper;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {

    private final List<Subsystem> mAllSubsystems;
    public List<Subsystem> getSubsystems(){ return mAllSubsystems; }

    private List<Loop> mLoops = new ArrayList<>();

    public SubsystemManager(List<Subsystem> allSubsystems) {
        mAllSubsystems = allSubsystems;
    }

    public void outputToSmartDashboard() {
        mAllSubsystems.forEach((s) -> s.outputTelemetry());
    }

    public void writeToLog() {
        mAllSubsystems.forEach((s) -> s.writeToLog());
    }

    public void stop() {
        mAllSubsystems.forEach((s) -> s.stop());
    }

    public boolean haveEmergency(){
        boolean emergency = false;
        for(Subsystem s : mAllSubsystems){
            emergency |= s.hasEmergency;
        }
        return emergency;
    }

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {
            for (Loop l : mLoops) {
                l.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem s : mAllSubsystems) {
               s.readPeriodicInputs();
            }
            for (Loop l : mLoops) {
                l.onLoop(timestamp);
            }
            for (Subsystem s : mAllSubsystems) {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(double timestamp) {
            for (Loop l : mLoops) {
                l.onStop(timestamp);
            }
        }
    }

    private class DisabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem s : mAllSubsystems) {
                s.readPeriodicInputs();
            }
            for (Subsystem s : mAllSubsystems) {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }
}

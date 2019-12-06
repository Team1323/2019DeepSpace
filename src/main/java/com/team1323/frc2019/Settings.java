/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * 
 */
public class Settings {

    private static Settings instance = new Settings(); 

    public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;

	public static final boolean kResetTalons = false;

    public static final boolean kSimulate = false;
    
    // Separate debugging output into the different subsystems so as to not 
    // overload the NetworkTables
    private boolean kDebugSwerve = false;
    private boolean kDebugElevator = false;
    private boolean kDebugIntakes = false;
    private boolean kDebugWrist = false;
    private boolean kDebugJacks = false;
    private boolean kDebugVision = false;

    private NetworkTableEntry swerveToggle;
    private NetworkTableEntry elevatorToggle;
    private NetworkTableEntry intakeToggle;
    private NetworkTableEntry wristToggle;
    private NetworkTableEntry jacksToggle;
    private NetworkTableEntry visionToggle;

    private final String TAB = "Settings";

    private void putToggles() {
        swerveToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Swerve", kDebugSwerve).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        elevatorToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Elevator", kDebugElevator).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        intakeToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Intakes", kDebugIntakes).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        wristToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Wrist", kDebugWrist).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        jacksToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Jacks", kDebugJacks).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        visionToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Vision", kDebugVision).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    }

    private void updateSettings() {
        instance.kDebugSwerve = swerveToggle.getBoolean(instance.kDebugSwerve);
        instance.kDebugElevator = elevatorToggle.getBoolean(instance.kDebugElevator);
        instance.kDebugIntakes = intakeToggle.getBoolean(instance.kDebugIntakes);
        instance.kDebugWrist = wristToggle.getBoolean(instance.kDebugWrist);
        instance.kDebugJacks = jacksToggle.getBoolean(instance.kDebugJacks);
        instance.kDebugVision = visionToggle.getBoolean(instance.kDebugVision);
    }

    public static void initializeToggles() {
        instance.putToggles();
    }

    public static void update() {
        instance.updateSettings();
    }

    public static boolean debugSwerve(){ return instance.kDebugSwerve; }
    public static boolean debugElevator(){ return instance.kDebugElevator; }
    public static boolean debugIntakes(){ return instance.kDebugIntakes; }
    public static boolean debugWrist(){ return instance.kDebugWrist; }
    public static boolean debugJacks(){ return instance.kDebugJacks; }
    public static boolean debugVision(){ return instance.kDebugVision; }

}

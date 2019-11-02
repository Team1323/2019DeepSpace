/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2019;

/**
 * 
 */
public class Settings {

    private static Settings instance = new Settings(); 

    public static final boolean kIsUsingCompBot = false;
	public static final boolean kIsUsingTractionWheels = true;

	public static final boolean kResetTalons = false;

    public static final boolean kSimulate = false;
    
    // Separate debugging output into the different subsystems so as to not 
    // overload the NetworkTables
    private boolean kDebugSwerve = false;
    private boolean kDebugElevator = true;
    private boolean kDebugIntakes = false;
    private boolean kDebugWrist = false;
    private boolean kDebugJacks = false;
    private boolean kDebugVision = false;

    public static boolean debugSwerve(){ return instance.kDebugSwerve; }
    public static boolean debugElevator(){ return instance.kDebugElevator; }
    public static boolean debugIntakes(){ return instance.kDebugIntakes; }
    public static boolean debugWrist(){ return instance.kDebugWrist; }
    public static boolean debugJacks(){ return instance.kDebugJacks; }
    public static boolean debugVision(){ return instance.kDebugVision; }

}

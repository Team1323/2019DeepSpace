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

    public static final boolean kIsUsingCompBot = true;
	public static final boolean kIsUsingTractionWheels = true;

	public static final boolean kResetTalons = false;

    public static final boolean kSimulate = true;
    
    // Separate debugging output into the different subsystems so as to not 
    // overload the NetworkTables
    public static final boolean kDebugSwerve = false;
    public static final boolean kDebugElevator = false;
    public static final boolean kDebugIntakes = false;
    public static final boolean kDebugWrist = false;
    public static final boolean kDebugJacks = false;
    public static final boolean kDebugVision = false;

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final double kTrackScrubFactor = 1.0469745223;


	public static final double kWheelDeadband = 0.04;
	public static final double kThrottleDeadband = 0.04;

	public static final double kTrackWidthInches = 24.2;
	public static final double kTrackWidthMeters = .6159;
	public static final double kWheelRadiusMeters = .0762;
	public static final double kMaxVelocityMetersPerSecond = 1;
	public static final double kMaxAccelerationMetersPerSecondSq = 2;
	public static final double kDriveGearRatio = 11.24;
	public static final int kTicksPerRotation = 2048;
	

	public static final double kDriveS = .327;  
	public static final double kDriveV = 2.53;  
	public static final double kDriveA =  0;  //0.168;  
	public static final double kDriveP = 0; 
	public static final double kDriveI = 0;
	public static final double kDriveD = 0;


	public static final int kPIDIdx = 0;
	public static final int kTimeoutMs = 10;

}

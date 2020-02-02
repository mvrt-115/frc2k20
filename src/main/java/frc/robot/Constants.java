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

	//Flywheel 
    public static final double kFlywheelP = 0.08;
    public static final double kFlywheelD = 0.8;
    public static final double kFlywheelFF = .0485;
    public static final double kFlywheelGearRatio = 20.0/34;
    public static final double kFlywheelAcceptableError = 100;

	//Joystick
    public static final double kSensitivity = 0.90;
	public static final double kWheelDeadband = 0.04;
	public static final double kThrottleDeadband = 0.04;

	//Drivetrain
    public static final double kTrackScrubFactor = 1.0469745223;
	public static final double kTrackWidthInches = 24.2;
	public static final double kTrackWidthMeters = .617432;
    public static final double kWheelDiameterMeters = .1450848;
    
	public static final double kMaxVelocityMetersPerSecond = 1;
	public static final double kMaxAccelerationMetersPerSecondSq = 2;
	public static final double kDriveGearRatio = (46.0/9) * (44.0/20);
	public static final int kFalconTicksPerRotation = 2048;
	
	public static final double kDriveS = 0.327; 
	public static final double kDriveV = 2.53; 
	public static final double kDriveA = 0.168;  

	public static final double kDriveP = 1; 
	public static final double kDriveI = 0;
	public static final double kDriveD = 0;

	//Intake
	public static final double kIntakeD = 0;
	public static final double kIntakeP = 0;
	public static final double kIntakeFF = 0;
	public static final double kIntakeStowedTicks = 0;
	public static final double kIntakeDeployTicks = 0;
	public static final double kIntakeMaxTicks = 1000;
	
	public static final int kPIDIdx = 0;
	public static final int kTimeoutMs = 10;
 
	//Climber
	public static final double kElevatorP = 0;
	public static final double kElevatorI = 0;
	public static final double kElevatorD = 0;
	public static final double kElevatorHoldVoltage = 0;
    public static final double kElevatorClimbVoltage = 0;
    
    public static final double kClimbHeight = 0;
    public static final double kLevelP = 0;
	public static final int kClimbTicks = 0;
}

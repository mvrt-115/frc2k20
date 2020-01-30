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

    public static double kFlywheelP = 0.076;
    public static double kFlywheelD = 0;
    public static double kFlywheelFF = .0465;

    public static double kFlywheelAcceptableError = 100;

    public static final int kPIDIdx = 0;
    public static final int kTimeoutMs = 10;
	public static final double kFlywheelGearRatio = 20.0/34;

}

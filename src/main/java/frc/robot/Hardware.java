/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.util.Limelight;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class Hardware {

    public static TalonFX flywheelMaster;
    public static TalonFX flywheelFollower;

    public static TalonFX leftMaster;
    public static TalonFX leftFollower;
    public static TalonFX rightMaster;
    public static TalonFX rightFollower;

    public static AHRS gyro;

    public static Limelight limelight;
    // intake
    public static TalonSRX intakeRoller;
    public static TalonSRX intakePivot;
    public static TalonSRX intakeFunnel;
    public static DigitalInput limitDown;
    public static DigitalInput limitUp;

}

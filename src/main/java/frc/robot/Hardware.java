/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utilities.Limelight;

/**
 * Add your docs here.
 */
public class Hardware {

    //drivetrain
    public static TalonFX frontLeft;
    public static TalonFX backLeft;
    public static TalonFX backRight;
    public static TalonFX frontRight;
    
    //hopper
    public static TalonSRX intake;
    public static TalonSRX hopper1;
    public static TalonSRX hopper2;
    public static TalonSRX hopper3;
    public static TalonSRX hopper4;
    public static DigitalInput breakbeamEnter;
    public static DigitalInput breakbeamExit;

    public static AHRS gyro;

    public static Limelight limelight;
 
}

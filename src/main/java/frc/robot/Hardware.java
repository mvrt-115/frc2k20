/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.Limelight;

/**
 * Add your docs here.
 */
public class Hardware {

  //Climber
  public static TalonFX elevatorMaster;
  public static TalonSRX levelMotor;
  public static DigitalInput elevatorBottomLimitSwitch;

  // Flywheel
  public static TalonFX flywheelMaster;
  public static TalonFX flywheelFollower;

  // Drivetrain
  public static TalonFX leftMaster;
  public static TalonFX leftFollower;
  public static TalonFX rightMaster;
  public static TalonFX rightFollower;

  // Hopper
  public static TalonSRX bottomHopper;
  public static TalonSRX topHopper;
  public static DigitalInput bottomHopperBreakbeam;
  public static DigitalInput topHopperBreakbeam;
  public static DigitalInput midHopperBreakbeam;

  // intake
  public static TalonSRX intakeRoller;
  public static TalonSRX intakePivot;
  public static TalonSRX intakeFunnel;
  public static DigitalInput intakeBottomlimitSwitch;

  public static AHRS gyro;
  public static Limelight limelight;

}

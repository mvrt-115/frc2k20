/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class Hopper extends SubsystemBase 
{
  public boolean canIntake;
  public boolean lastEnter;
  public int balls;
  /**
   * Creates a new Hopper.
   */
  public Hopper() 
  {
    Hardware.intake = new TalonSRX(0);
    Hardware.hopper1 = new TalonSRX(1);
    Hardware.hopper2 = new TalonSRX(2);
    Hardware.hopper3 = new TalonSRX(3);
    Hardware.hopper4 = new TalonSRX(4);
    Hardware.breakbeamEnter = new DigitalInput(0);
    Hardware.breakbeamExit = new DigitalInput(1);
    balls = 0;
    canIntake = true;
    lastEnter = false;
  }

  public void intake()
  {
    if(canIntake)
    {
      Hardware.intake.set(ControlMode.PercentOutput, 1);
      
      Hardware.hopper1.set(ControlMode.PercentOutput, 1);
      Hardware.hopper2.set(ControlMode.PercentOutput, 1);
      Hardware.hopper3.set(ControlMode.PercentOutput, 1);
      Hardware.hopper4.set(ControlMode.PercentOutput, 1);
    }

    if(lastEnter && !Hardware.breakbeamEnter.get()) 
    {
      balls++;
      Timer.delay(0.5);
    }

    lastEnter = Hardware.breakbeamEnter.get();
    if(balls == 5) canIntake = false;
    else canIntake = true;
  }

  public void stop()
  {
    Hardware.intake.set(ControlMode.PercentOutput, 0);
    Hardware.hopper1.set(ControlMode.PercentOutput, 0);
    Hardware.hopper2.set(ControlMode.PercentOutput, 0);
    Hardware.hopper3.set(ControlMode.PercentOutput, 0);
    Hardware.hopper4.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}

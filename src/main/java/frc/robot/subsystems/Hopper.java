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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class Hopper extends SubsystemBase 
{
  public boolean canIntake;
  /**
   * Creates a new Hopper.
   */
  public Hopper() 
  {
    Hardware.intake = new TalonSRX(0);
    Hardware.hopperOne = new TalonSRX(1);
    Hardware.hopperTwo = new TalonSRX(2);
    Hardware.breakbeamOne = new DigitalInput(0);
    Hardware.breakbeamTwo = new DigitalInput(1);
    canIntake = true;
  }

  public void intake()
  {
    if(canIntake)
    {
      Hardware.intake.set(ControlMode.PercentOutput, 1);

      Hardware.hopperOne.set(ControlMode.PercentOutput, 1);
      Hardware.hopperOne.set(ControlMode.PercentOutput, 1);
    }
    if(Hardware.breakbeamOne.get() && Hardware.breakbeamTwo.get()) canIntake = false;
    else if(Hardware.breakbeamOne.get() || Hardware.breakbeamTwo.get()) canIntake = true;
  }

  public void stop()
  {
    Hardware.hopperOne.set(ControlMode.PercentOutput, 0);
    Hardware.hopperTwo.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}

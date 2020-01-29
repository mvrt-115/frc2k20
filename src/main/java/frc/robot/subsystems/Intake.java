/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class Intake extends SubsystemBase 
{

  public enum IntakeState {RETRACTED, DEPLOYED};
  public IntakeState currState = IntakeState.RETRACTED;
  /**
   * Creates a new Intake.
   */
  public Intake() 
  {
    Hardware.intakeRoller = new TalonSRX(0);
    Hardware.intakePivot = new TalonSRX(1);
    Hardware.intakeFunnel = new TalonSRX(2);
  }

  public void setRoller(double rate)
  {
    Hardware.intakeRoller.set(ControlMode.PercentOutput, rate); 
  }

  public void setPivot(double rate)
  {
    Hardware.intakePivot.set(ControlMode.PercentOutput, rate); 
  }

  public void setFunnel(double rate)
  {
    Hardware.intakeFunnel.set(ControlMode.PercentOutput, rate); 
  }

  public double getDirection()
  {
    if(currState == IntakeState.RETRACTED) return 1;
    return -1;
  }

  public void changePosition()
  {
    if(currState == IntakeState.RETRACTED) currState = IntakeState.DEPLOYED;
    else currState = IntakeState.RETRACTED;
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}

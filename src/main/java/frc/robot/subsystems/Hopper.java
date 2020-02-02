/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class Hopper extends SubsystemBase 
{
 
  /**
   * Creates a new Hopper.
   */
  public Hopper() 
  {
    Hardware.bottomHopper = new TalonSRX(22);
    Hardware.topHopper = new TalonSRX(32);

    Hardware.bottomHopper.configFactoryDefault();
    Hardware.topHopper.configFactoryDefault();

    Hardware.bottomHopper.setInverted(false);
    Hardware.topHopper.setInverted(false);

    Hardware.topHopper.setNeutralMode(NeutralMode.Brake);
    Hardware.bottomHopper.setNeutralMode(NeutralMode.Brake);

    Hardware.bottomHopperBreakbeam = new DigitalInput(0);
    Hardware.TopHopperBreakbeam = new DigitalInput(1);

  }

  public void runHopper() {
    Hardware.bottomHopper.set(ControlMode.PercentOutput, 0.3);
    Hardware.topHopper.set(ControlMode.PercentOutput, 0.3);
  }

  public void runBottom(){
    Hardware.bottomHopper.set(ControlMode.PercentOutput, 0.3);
  }

  public boolean getBottomBreakbeam() {
    return Hardware.bottomHopperBreakbeam.get();
  }

  public boolean getTopBreakbeam(){
     return Hardware.TopHopperBreakbeam.get();
   }

  public void stopMotors(){
    Hardware.topHopper.set(ControlMode.PercentOutput, 0);
    Hardware.bottomHopper.set(ControlMode.PercentOutput,0);
  }
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}

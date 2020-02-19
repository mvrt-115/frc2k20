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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;

public class Hopper extends SubsystemBase 
{
 
  /**
   * Creates a new Hopper.
   */
  private int balls;
  private boolean lastBottom;
  private boolean lastTop;

  public Hopper() 
  {
    Hardware.bottomHopper = new TalonSRX(35);
    Hardware.topHopper = new TalonSRX(32);

    Hardware.bottomHopper.configFactoryDefault();
    Hardware.topHopper.configFactoryDefault();

    Hardware.bottomHopper.setInverted(false);
    Hardware.topHopper.setInverted(true);

    Hardware.topHopper.setNeutralMode(NeutralMode.Brake);
    Hardware.bottomHopper.setNeutralMode(NeutralMode.Brake);

    Hardware.bottomHopperBreakbeam = new DigitalInput(2);
    Hardware.midHopperBreakbeam = new DigitalInput(1);
    Hardware.topHopperBreakbeam = new DigitalInput(0);
    balls = 0;
    lastBottom = false;
    lastTop = false;
  }

  public void runHopper(double _topSpeed, double _botSpeed) {
    Hardware.bottomHopper.set(ControlMode.PercentOutput, _botSpeed);
    Hardware.topHopper.set(ControlMode.PercentOutput, _topSpeed);
  }

  public void runBottom(){
    Hardware.bottomHopper.set(ControlMode.PercentOutput, 0.3);
  }

  public boolean getBottomBreakbeam() {
    return !Hardware.bottomHopperBreakbeam.get();
  }

  public boolean getTopBreakbeam(){
     return !Hardware.topHopperBreakbeam.get();
   }

   public boolean getMiddleBreakbeam(){
     return !Hardware.midHopperBreakbeam.get();
   }

  public void stopMotors(){
    Hardware.topHopper.set(ControlMode.PercentOutput, 0);
    Hardware.bottomHopper.set(ControlMode.PercentOutput,0);
  }
  @Override
  public void periodic() 
  {
    boolean currBottom = getBottomBreakbeam();
    boolean currTop = getTopBreakbeam();

    if(Robot.getRobotState() != RobotState.DISABLED){
      if(currBottom && !lastBottom)
        balls++;

      if(!currTop && lastTop)
        balls--;
    }
    lastTop = currTop;
    lastBottom = currBottom;
  }

  public int getBalls(){
    return balls;
  }

  public void setBalls(int _balls){
    balls = _balls;
  }


  public void log(){
  //  SmartDashboard.putBoolean("bottom Breakbeam", getBottomBreakbeam());
   SmartDashboard.putBoolean("Top Breakbeam", getTopBreakbeam());
    SmartDashboard.putNumber("Num of Balls", balls);
    SmartDashboard.putNumber("Output", Hardware.topHopper.getMotorOutputPercent());
  }
}

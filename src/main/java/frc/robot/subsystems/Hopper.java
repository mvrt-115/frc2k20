/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.Flywheel.FlywheelState;
import frc.robot.subsystems.LEDStrip.LEDColor;

public class Hopper extends SubsystemBase {

  /**
   * Creates a new Hopper.
   */
  private int balls;
  private boolean lastBottom;
  private boolean lastTop;
  private SupplyCurrentLimitConfiguration currentConfig;
  private double lastTimeBottom;
  private double lastTimeTop;
  private int topTicks;
  private int botTicks;
  private static int kdebounce_max = 10;
  private static int kdebounce_trigger_top = 7;
  private static int kdebounce_trigger_bot = 4;


  public Hopper() {

    if(Constants.kCompBot){   
      Hardware.bottomHopper = new TalonSRX(2);
      Hardware.topHopper = new TalonSRX(7);
      Hardware.bottomHopperBreakbeam = new DigitalInput(2);
      Hardware.topHopperBreakbeam = new DigitalInput(6);
      Hardware.midHopperBreakbeam = new DigitalInput(7);
    }else{
      Hardware.bottomHopper = new TalonSRX(35);
      Hardware.topHopper = new TalonSRX(32);
      Hardware.bottomHopperBreakbeam = new DigitalInput(2);
      Hardware.topHopperBreakbeam = new DigitalInput(0);
    }

    Hardware.bottomHopper.configFactoryDefault();
    Hardware.topHopper.configFactoryDefault();

    Hardware.bottomHopper.setInverted(false);
    Hardware.topHopper.setInverted(false);

    Hardware.bottomHopper.configVoltageCompSaturation(10, Constants.kTimeoutMs);
    Hardware.topHopper.configVoltageCompSaturation(10, Constants.kTimeoutMs);

    Hardware.bottomHopper.enableVoltageCompensation(true);
    Hardware.topHopper.enableVoltageCompensation(true);


    currentConfig = new SupplyCurrentLimitConfiguration(true, 30, 1.3, 18);

    Hardware.bottomHopper.configSupplyCurrentLimit(currentConfig, Constants.kTimeoutMs);
    Hardware.topHopper.configSupplyCurrentLimit(currentConfig, Constants.kTimeoutMs);

    Hardware.topHopper.setNeutralMode(NeutralMode.Brake);
    Hardware.bottomHopper.setNeutralMode(NeutralMode.Brake);

    balls = 0;
    lastBottom = false;
    lastTop = false;
    botTicks = 0;
    topTicks = 0;
    lastTimeBottom = Timer.getFPGATimestamp();
    lastTimeTop = Timer.getFPGATimestamp();
  }

  public void runHopper(double _topSpeed, double _botSpeed) {
    Hardware.bottomHopper.set(ControlMode.PercentOutput, _botSpeed);
    Hardware.topHopper.set(ControlMode.PercentOutput, _topSpeed);
  }

  public void runBottom() {
    Hardware.bottomHopper.set(ControlMode.PercentOutput, 0.3);
  }

  public boolean getBottomBreakbeam() {
    return !Hardware.bottomHopperBreakbeam.get();
  }

  public boolean getTopBreakbeam() {
    return !Hardware.topHopperBreakbeam.get();
  }

  public boolean getMidBreakbeam(){
    return !Hardware.midHopperBreakbeam.get();
  }

  public void stopMotors() {
    Hardware.topHopper.set(ControlMode.PercentOutput, 0);
    Hardware.bottomHopper.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

    boolean currTop = getTopBreakbeam();
    boolean currBottom = getBottomBreakbeam();

   /* if(getTopBreakbeam()){
      if(topTicks < kdebounce_max) topTicks++;
      if(topTicks>= kdebounce_trigger_top) currTop = true;
    }else{
      if(topTicks > 0) topTicks--;
      if(topTicks <= kdebounce_trigger_bot) currTop = false;
    }


    if(getBottomBreakbeam()){
      if(botTicks < kdebounce_max) botTicks ++;
      if(botTicks >= kdebounce_trigger_top) currBottom = true;
    }else{
      if(botTicks > 0) botTicks--;
      if(botTicks <= kdebounce_trigger_bot) currBottom = false;
    }
*/

    if (Robot.getRobotState() != RobotState.DISABLED) {
      if (currBottom && !lastBottom)
        if(Timer.getFPGATimestamp() - lastTimeBottom > .3){
          balls++;
          lastTimeBottom = Timer.getFPGATimestamp();
        }
      if (!currTop && lastTop && (Robot.flywheel.getFlywheelState() == FlywheelState.SPINNINGUP ||Robot.flywheel.getFlywheelState() == FlywheelState.ATSPEED) ){
       if(Timer.getFPGATimestamp() - lastTimeTop > .15){
          lastTimeTop = Timer.getFPGATimestamp();
          balls--;
       }
      }
    }

    if(balls <0 || balls > 5){
      Robot.led.setColor(LEDColor.RED);
    }
    lastTop = currTop;
    lastBottom = currBottom;

     if(balls > 6){
        setBalls(5);
     } 

     if(balls < 0){
       setBalls(0);
     }

  }

  public int getBalls() {
    return balls;
  }

  public void setBalls(int _balls) {
    balls = _balls;
  }

  public void log() {
     SmartDashboard.putBoolean("bottom Breakbeam", getBottomBreakbeam());
     SmartDashboard.putBoolean("Top Breakbeam", getTopBreakbeam());
    SmartDashboard.putNumber("Num of Balls", balls);

    SmartDashboard.putNumber("Bottom Current", Hardware.bottomHopper.getSupplyCurrent());
    // SmartDashboard.putNumber("Output", Hardware.topHopper.getMotorOutputPercent());
  }
}

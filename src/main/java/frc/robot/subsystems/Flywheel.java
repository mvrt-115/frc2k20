/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.util.RollingAverage;

public class Flywheel extends SubsystemBase {
  /**
   * Creates a new Flywheel.
   */
  public enum FlywheelState {
    OFF, SPINNINGUP, ATSPEED;
  }

  private FlywheelState currState;
  private CANPIDController velocityController;
  private CANEncoder flyWheelEncoder;
  private double targetVelocity;
  private RollingAverage flywheelRPM;

  double time1, time2;

  public Flywheel() {
    Hardware.flywheelMaster = new CANSparkMax(7, MotorType.kBrushless);
    Hardware.flywheelFollower = new CANSparkMax(4, MotorType.kBrushless);

    Hardware.flywheelMaster.restoreFactoryDefaults();
    Hardware.flywheelFollower.restoreFactoryDefaults();

    Hardware.flywheelMaster.setInverted(false);
    Hardware.flywheelFollower.follow(Hardware.flywheelMaster, true);

    flyWheelEncoder = Hardware.flywheelMaster.getEncoder();
    velocityController = Hardware.flywheelMaster.getPIDController();

    velocityController.setP(Constants.kFlywheelP);
    velocityController.setD(Constants.kFlywheelD);
    velocityController.setFF(Constants.kFlywheelFF);
    velocityController.setOutputRange(-1, 1);

    targetVelocity = 0;
    currState = FlywheelState.OFF;

    flywheelRPM = new RollingAverage(5);

    time1 = 0;
    time2 = 0;
  }

  public void setTargetVelocity(double desiredVelocity) {
    targetVelocity = desiredVelocity;
    currState = FlywheelState.SPINNINGUP;
  }

  public void log() {
    SmartDashboard.putNumber("Motor RPM", getMotorRPM());
    SmartDashboard.putNumber("Motor Output", Hardware.flywheelMaster.getAppliedOutput());
    SmartDashboard.putNumber("Target Velocity", targetVelocity);
    // SmartDashboard.putString("STATE", currState.toString());

  }

  public double getMotorRPM() {
    return flyWheelEncoder.getVelocity();
  }

  public void setFlywheelState(FlywheelState newState){
    currState = newState;
  }

  public void periodic() {
    
    flywheelRPM.add(getMotorRPM());

    switch (currState) {

    case OFF:
      SmartDashboard.putString("STATE", "OFF");
      Hardware.flywheelMaster.set(0);

      break;
    case SPINNINGUP:
      SmartDashboard.putString("STATE", "Spinning UP");
      velocityController.setReference(targetVelocity, ControlType.kVelocity);

      if(flywheelRPM.withinError(targetVelocity, Constants.kFlywheelAcceptableError)){
        time1 = Timer.getFPGATimestamp();
        setFlywheelState(FlywheelState.ATSPEED);
      }
      break;
    case ATSPEED:
      SmartDashboard.putString("STATE", "AT SPEED");
      velocityController.setReference(targetVelocity, ControlType.kVelocity);
   
      if(!flywheelRPM.withinError(targetVelocity, Constants.kFlywheelAcceptableError)){
        time2 = Timer.getFPGATimestamp();
        setFlywheelState(FlywheelState.SPINNINGUP);
      }

      break;
    }


    double recoverTime = time1-time2;
    SmartDashboard.putNumber("RecoverTime", recoverTime);
  }
}

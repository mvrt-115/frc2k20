/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;

public class Intake extends SubsystemBase 
{

  public enum IntakeState {
    STOWED, DEPLOYED, STOWING, DEPLOYING, INTAKING
  };

  private IntakeState currState;
  /**
   * Creates a new Intake.
   */
  public Intake() 
  {
    Hardware.intakeRoller = new TalonSRX(0);
    Hardware.intakePivot = new TalonSRX(1);
    Hardware.intakeFunnel = new TalonSRX(2);

    Hardware.intakeRoller.configFactoryDefault();
    Hardware.intakePivot.configFactoryDefault();
    Hardware.intakeFunnel.configFactoryDefault();

    Hardware.intakeRoller.setInverted(false);
		Hardware.intakePivot.setInverted(false);
		Hardware.intakeFunnel.setInverted(true);
  

    Hardware.intakePivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDIdx, Constants.kTimeoutMs);
    Hardware.intakePivot.setSelectedSensorPosition(0);

    Hardware.intakePivot.config_kP(Constants.kPIDIdx, Constants.kIntakeP);
    Hardware.intakePivot.config_kD(Constants.kPIDIdx, Constants.kIntakeD);
    currState = IntakeState.STOWED;
  }

  public void runIntake(){
    Hardware.intakeFunnel.set(ControlMode.PercentOutput, 0.3);
    Hardware.intakeRoller.set(ControlMode.PercentOutput, 0.4);
  }

  public void setIntakeState(IntakeState desiredState){
    currState = desiredState; 
  }

  public IntakeState getIntakeState(){
    return currState;
  }

  public int getPivotTicks(){
    return Hardware.intakePivot.getSelectedSensorPosition();
  }

  
  public void periodic() 
  {
    switch(currState){
      
      case STOWED:
          Hardware.intakePivot.set(ControlMode.PercentOutput, 0);

      break;
      case DEPLOYED:
         Hardware.intakePivot.set(ControlMode.PercentOutput, 0);
      break;
      case DEPLOYING:
        Hardware.intakePivot.set(ControlMode.MotionMagic, Constants.kIntakeDeployTicks);

        if(getPivotTicks() > Constants.kIntakeDeployTicks)
          currState = IntakeState.DEPLOYED;

      break;
      case STOWING:
        Hardware.intakePivot.set(ControlMode.MotionMagic, Constants.kIntakeStowedTicks);

        if(getPivotTicks() < Constants.kIntakeStowedTicks)
        currState = IntakeState.DEPLOYED;

      break;
      case INTAKING:
        Hardware.intakePivot.set(ControlMode.PercentOutput, 0);
        runIntake();

      break;
    }
  }
}

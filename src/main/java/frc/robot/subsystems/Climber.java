/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Elevator.
   */

  private ElevatorState currState;
  private double angleError;

  public enum ElevatorState {
    PULLING, SETPOINT, HOLD, ZEROED, LEVEL, ZEROING
  };

  public Climber() {

    Hardware.elevatorMaster = new TalonFX(0);
    Hardware.levelMotor = new TalonSRX(1);

    Hardware.elevatorMaster.configFactoryDefault();
    Hardware.levelMotor.configFactoryDefault();

    Hardware.elevatorMaster.setInverted(false);
    Hardware.levelMotor.setInverted(false);

    Hardware.elevatorMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
        Constants.kTimeoutMs);

    Hardware.elevatorMaster.setSelectedSensorPosition(0);

    currState = ElevatorState.ZEROED;

    Hardware.elevatorMaster.configPeakOutputForward(1);
    Hardware.elevatorMaster.configPeakOutputReverse(-1);

    Hardware.elevatorMaster.config_kP(Constants.kPIDIdx, Constants.kElevatorP);
    Hardware.elevatorMaster.config_kI(Constants.kPIDIdx, Constants.kElevatorI);
    Hardware.elevatorMaster.config_kD(Constants.kPIDIdx, Constants.kElevatorD);

  }

  public void periodic() {

    switch (currState) {

    case SETPOINT:
      Hardware.elevatorMaster.set(ControlMode.Position, Constants.kClimbHeight, DemandType.ArbitraryFeedForward, Constants.kElevatorHoldVoltage/12);
      if(Math.abs(Hardware.elevatorMaster.getSelectedSensorPosition() - Constants.kClimbTicks) < Constants.kClimbThreshold){
        currState = ElevatorState.HOLD;
      }
      break;

    case HOLD:
      break;
    
    case PULLING:
      Hardware.elevatorMaster.set(ControlMode.Position, Constants.kClimbZero, DemandType.ArbitraryFeedForward, Constants.kElevatorHoldVoltage/12);
      if(Math.abs(Hardware.elevatorMaster.getSelectedSensorPosition() - Constants.kClimbZero) < Constants.kClimbThreshold){
        currState = ElevatorState.LEVEL;
      }
      break;
    
    case LEVEL:
      Hardware.levelMotor.set(ControlMode.PercentOutput, angleError * Constants.kLevelP);
      break;

    case ZEROING:
      Hardware.elevatorMaster.set(ControlMode.Position, Constants.kClimbZero, DemandType.ArbitraryFeedForward, Constants.kElevatorHoldVoltage/12);
      if(Math.abs(Hardware.elevatorMaster.getSelectedSensorPosition() - Constants.kClimbZero) < Constants.kClimbThreshold){
        currState = ElevatorState.ZEROED;
      }
      break;

    case ZEROED:
      stop();
      break;
    }
  }

  public void stop() {
    Hardware.elevatorMaster.set(ControlMode.PercentOutput, 0);
  }

  public void setElevatorState(ElevatorState desiredState){
    currState = desiredState;
  }

  public ElevatorState getElevatorState(){
    return currState;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.LEDStrip.LEDColor;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Elevator.
   */

  private ElevatorState currState;
  private SupplyCurrentLimitConfiguration currentConfig;

  public enum ElevatorState {
    ZEROED, SETPOINT, CLIMBING, HOLD, ZEROING
  };

  public Climber() {

    if(Constants.kCompBot){
      Hardware.elevatorMaster = new TalonFX(22);
    }else{
      Hardware.elevatorMaster = new TalonFX(9);
    }
    
    Hardware.elevatorServo = new Servo(0);
    Hardware.elevatorMaster.configFactoryDefault();

    Hardware.elevatorMaster.setInverted(Constants.kCompBot);

    Hardware.elevatorMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
        Constants.kTimeoutMs);

    Hardware.elevatorMaster.setSelectedSensorPosition(0);


    
    currentConfig = new SupplyCurrentLimitConfiguration(true, 40, 1.3, 30);

    Hardware.bottomHopper.configSupplyCurrentLimit(currentConfig, Constants.kTimeoutMs);
    Hardware.topHopper.configSupplyCurrentLimit(currentConfig, Constants.kTimeoutMs);


    currState = ElevatorState.ZEROED;

    Hardware.elevatorMaster.configPeakOutputForward(1);
    Hardware.elevatorMaster.configPeakOutputReverse(-1);

    Hardware.elevatorMaster.config_kP(Constants.kPIDIdx, Constants.kElevatorP);
    Hardware.elevatorMaster.config_kI(Constants.kPIDIdx, Constants.kElevatorI);
    Hardware.elevatorMaster.config_kD(Constants.kPIDIdx, Constants.kElevatorD);
  }

  public void periodic() {

    switch (currState) {

    case ZEROED:     
     if(Robot.drivetrain.handleDeadband(Robot.oi.operatorJoystick.getRawAxis(5), .1) != 0){
      Robot.intake.setIntakeState(IntakeState.DEPLOYING);
     // Hardware.elevatorServo.set(Constants.kServoUnRatchet);
     }
     
     Hardware.elevatorMaster.set(ControlMode.PercentOutput, -Robot.drivetrain.handleDeadband(Robot.oi.operatorJoystick.getRawAxis(5), .05));
      SmartDashboard.putString("Climber State", "ZEROED");

      break;
    case SETPOINT:
    Robot.led.setColor(LEDColor.RAINBOW);
  //  Hardware.elevatorServo.set(Constants.kServoUnRatchet);
    Robot.intake.setIntakeState(IntakeState.DEPLOYING);
    
      Hardware.elevatorMaster.set(ControlMode.Position, Constants.kClimbHeight, DemandType.ArbitraryFeedForward,
          Constants.kElevatorHoldOutput);

      SmartDashboard.putString("Climber State", "SETPOINT");
      break;

     
    case CLIMBING:
    Hardware.elevatorServo.set(Constants.kServoRatchet);
      Hardware.elevatorMaster.set(ControlMode.PercentOutput, Constants.kElevatorClimbOutput);

      if (Hardware.elevatorMaster.getSelectedSensorPosition() < Constants.kClimbTicks) {
        currState = ElevatorState.HOLD;
      }

      SmartDashboard.putString("Climber State", "CLIMBING");
      break;
    case HOLD:
      Hardware.elevatorServo.set(Constants.kServoRatchet);
      Hardware.elevatorMaster.set(ControlMode.PercentOutput, 0);

      SmartDashboard.putString("Climber State", "HOLD");
      break;
      
    case ZEROING:
      Hardware.elevatorMaster.set(ControlMode.Position, Constants.kElevatorZero, DemandType.ArbitraryFeedForward,
          Constants.kElevatorHoldOutput);

      if(getEncoder() < 4000)
        setElevatorState(ElevatorState.ZEROED);
      
      break;
    }    
  }

  public double getEncoder(){
    return Hardware.elevatorMaster.getSelectedSensorPosition();
  }
  public boolean getLimitSwitch(){
    return Hardware.elevatorBottomLimitSwitch.get();
  }

  public void resetEncoder() {
    Hardware.elevatorMaster.setSelectedSensorPosition(0);
  }

  public void setElevatorState(ElevatorState desiredState) {
    currState = desiredState;
  }

  public ElevatorState getElevatorState() {
    return currState;
  }

  public void log() {
    SmartDashboard.putNumber("Climber encoder", getEncoder());
    SmartDashboard.putNumber("Servo Position", Hardware.elevatorServo.get());
  }
}

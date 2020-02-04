/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;

public class Intake extends SubsystemBase {

  public enum IntakeState {
    STOWED, DEPLOYED, STOWING, DEPLOYING, INTAKING
  };

  private IntakeState currState;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    Hardware.intakeRoller = new TalonSRX(36);
    Hardware.intakePivot = new TalonSRX(30);
    Hardware.intakeFunnel = new TalonSRX(31);

    Hardware.intakeBottomlimitSwitch = new DigitalInput(5);

    Hardware.intakeRoller.configFactoryDefault();
    Hardware.intakePivot.configFactoryDefault();
    Hardware.intakeFunnel.configFactoryDefault();

    Hardware.intakeRoller.setInverted(false);
    Hardware.intakePivot.setInverted(false);
    Hardware.intakeFunnel.setInverted(false);

    Hardware.intakeRoller.setNeutralMode(NeutralMode.Coast);
    Hardware.intakePivot.setNeutralMode(NeutralMode.Brake);
    Hardware.intakeFunnel.setNeutralMode(NeutralMode.Coast);

    Hardware.intakePivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDIdx,
        Constants.kTimeoutMs);

    Hardware.intakePivot.setSensorPhase(false);
    Hardware.intakePivot.setSelectedSensorPosition(0);

    Hardware.intakePivot.config_kP(Constants.kPIDIdx, Constants.kIntakeP);
    Hardware.intakePivot.config_kD(Constants.kPIDIdx, Constants.kIntakeD);
    currState = IntakeState.STOWED;
  }

  public void runIntake() {
    Hardware.intakeFunnel.set(ControlMode.PercentOutput, 0.3);
    Hardware.intakeRoller.set(ControlMode.PercentOutput, 0.4);
  }

  public void periodic() {
    switch (currState) {

    case STOWED:
      SmartDashboard.putString("INTAKE STATE", "Stowed");
      Hardware.intakePivot.set(ControlMode.PercentOutput, 0);

      break;
    case DEPLOYED:
      SmartDashboard.putString("INTAKE STATE", "Deployed");
      Hardware.intakePivot.set(ControlMode.PercentOutput, 0);
      break;
    case DEPLOYING:
      SmartDashboard.putString("INTAKE STATE", "Deploying");
      Hardware.intakePivot.set(ControlMode.MotionMagic, Constants.kIntakeDeployTicks, DemandType.ArbitraryFeedForward,
          Constants.kIntakeFF * getIntakePivotAngle());

      if (getPivotTicks() > Constants.kIntakeDeployTicks)
        currState = IntakeState.DEPLOYED;

      break;
    case STOWING:
      SmartDashboard.putString("INTAKE STATE", "Stowing");
      Hardware.intakePivot.set(ControlMode.MotionMagic, Constants.kIntakeStowedTicks, DemandType.ArbitraryFeedForward,
          Constants.kIntakeFF * getIntakePivotAngle());

      if (getPivotTicks() < Constants.kIntakeStowedTicks)
        currState = IntakeState.DEPLOYED;

      break;
    case INTAKING:
      SmartDashboard.putString("INTAKE STATE", "Intaking");
      Hardware.intakePivot.set(ControlMode.PercentOutput, 0);
      runIntake();

      break;
    }
  }

  public void setIntakeState(IntakeState desiredState) {
    currState = desiredState;
  }

  public IntakeState getIntakeState() {
    return currState;
  }

  public void log() {
    SmartDashboard.putNumber("Intake Encoder", Hardware.intakePivot.getSelectedSensorPosition());
  }

  public int getPivotTicks() {
    return Hardware.intakePivot.getSelectedSensorPosition();
  }

  public double getIntakePivotAngle() {
    return Hardware.intakePivot.getSelectedSensorPosition() / Constants.kIntakeMaxTicks * Math.PI / 2;
  }

  public boolean getBottomLimitSwitch() {
    return Hardware.intakeBottomlimitSwitch.get();
  }

  public void resetPivotEncoder() {
    Hardware.intakePivot.setSelectedSensorPosition(0);
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeState;

public class ManualHopper extends CommandBase {
  /**
   * Creates a new ManualHopper.
   */
  double bottomSpeed, topSpeed;

  public ManualHopper(double _bottomSpeed, double _topSpeed) {
    addRequirements(Robot.hopper);
    bottomSpeed = _bottomSpeed;
    topSpeed = _topSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.intake.setIntakeState(IntakeState.DEPLOYING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.hopper.runHopper(topSpeed, bottomSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.hopper.stopMotors();
    Robot.intake.setIntakeState(IntakeState.STOWING);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Robot.oi.getHopperButton();
  }
}

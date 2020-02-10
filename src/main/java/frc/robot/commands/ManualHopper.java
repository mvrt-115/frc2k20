/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;
import frc.robot.Robot;

public class ManualHopper extends CommandBase {
  /**
   * Creates a new ManualHopper.
   */
  public ManualHopper() {
    addRequirements(Robot.hopper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.hopper.runHopper(.5, .4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.hopper.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Robot.oi.getHopperButton();
  }
}

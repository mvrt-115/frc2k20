/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveWithJoystick extends CommandBase 
{
  /**
   * Creates a new DriveWithJoystick.
   */
  public DriveWithJoystick() 
  {
    addRequirements(Robot.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
   Robot.drivetrain.cheesyIshDrive(Robot.oi.getThrottle(), Robot.oi.getWheel(), Robot.oi.getQuickTurn());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    Robot.drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
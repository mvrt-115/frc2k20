/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.util.RollingAverage;
import frc.robot.util.Limelight.LED_MODE;
import frc.robot.util.Limelight.PIPELINE_STATE;

public class AutoAlign extends CommandBase {
  /**
   * Creates a new AutoAlign.
   */
  RollingAverage horizontalOffset;

  public AutoAlign() {
    horizontalOffset = new RollingAverage(5);
    addRequirements(Robot.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    horizontalOffset.reset();
    Hardware.limelight.setPipeline(PIPELINE_STATE.VISION_WIDE);
    Hardware.limelight.setLED(LED_MODE.ON);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    horizontalOffset.add(Hardware.limelight.getHorizontalAngle());
    Robot.drivetrain.alignToTarget(horizontalOffset.getAverage());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.stop();
    Hardware.limelight.setPipeline(PIPELINE_STATE.DRIVER);
    Hardware.limelight.setLED(LED_MODE.OFF);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Robot.oi.getAlign();
  }
}
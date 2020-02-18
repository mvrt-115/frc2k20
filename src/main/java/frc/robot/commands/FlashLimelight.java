/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.util.Limelight.LED_MODE;

public class FlashLimelight extends CommandBase {
  /**
   * Creates a new FlashLimelight.
   */
  private double startTime;
  private double timeoutSeconds;
  public FlashLimelight(double timeoutSeconds) {
    this.timeoutSeconds = timeoutSeconds;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    Hardware.limelight.setLED(LED_MODE.BLINKING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hardware.limelight.setLED(LED_MODE.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - startTime > timeoutSeconds){
      return true;
    }
    return false;
  }
}
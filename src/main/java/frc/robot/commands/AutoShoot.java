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
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.Flywheel.FlywheelState;
import frc.robot.util.RollingAverage;
import frc.robot.util.Limelight.LED_MODE;
import frc.robot.util.Limelight.PIPELINE_STATE;

public class AutoShoot extends CommandBase {
  /**
   * Creates a new autoShoot.
   */

  private double timeout;
  private double startTime;
  private RollingAverage horizontalOffset;

  public AutoShoot(double timeoutSeconds) {
    timeout = timeoutSeconds;
    horizontalOffset = new RollingAverage(5);
    startTime = 0;

    addRequirements(Robot.drivetrain);
    addRequirements(Robot.flywheel);
    addRequirements(Robot.hopper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    horizontalOffset.reset();

    Hardware.limelight.setPipeline(PIPELINE_STATE.VISION_ZOOM);
    Hardware.limelight.setLED(LED_MODE.ON);
    
    Robot.flywheel.setTargetVelocity(3000);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    horizontalOffset.add(Hardware.limelight.getHorizontalAngle());
    Robot.drivetrain.alignToTarget(horizontalOffset.getAverage());

    Robot.flywheel.setTargetVelocity(0);


    if (Hardware.limelight.hasTarget() && Robot.flywheel.getFlywheelState() == FlywheelState.ATSPEED
        && horizontalOffset.allWithinError(0, .5)) {
      Robot.hopper.runHopper();
    }else{
      if(!Robot.hopper.getTopBreakbeam()){
          Robot.hopper.runHopper();
      }
    }

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

    if (Robot.getRobotState() == RobotState.TELEOP) {
      if (!Robot.oi.getAlignButton()) {
        return true;
      }
      return false;
    } else {
      if (Timer.getFPGATimestamp() - startTime < timeout) {
        return true;
      }
      return false;
    }

  }
}

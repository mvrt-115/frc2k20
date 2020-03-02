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
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.Flywheel.FlywheelState;
import frc.robot.subsystems.LEDStrip.LEDColor;
import frc.robot.util.RollingAverage;
import frc.robot.util.Limelight.LED_MODE;
import frc.robot.util.Limelight.PIPELINE_STATE;

public class AutoShoot extends CommandBase {
  /**
   * Creates a new autoShoot.
   */

  private double RPM;
  private RollingAverage horizontalOffset;
  private RollingAverage verticalOffset;

  public AutoShoot(double _RPM) {
    
    RPM = _RPM;
    horizontalOffset = new RollingAverage(12);
    verticalOffset = new RollingAverage(5);

    addRequirements(Robot.drivetrain);
    addRequirements(Robot.flywheel);
    addRequirements(Robot.hopper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.integralAcc = 0;
    horizontalOffset.reset();
    verticalOffset.reset();
    Robot.flywheel.setFlywheelState(FlywheelState.SPINNINGUP);
    Hardware.limelight.setPipeline(PIPELINE_STATE.VISION_WIDE);
    Hardware.limelight.setLED(LED_MODE.ON);
    Robot.led.setColor(LEDColor.YELLOW);
    
    if(RPM != 0){
      Robot.flywheel.setTargetVelocity(RPM);
    }else{
      Robot.flywheel.setTargetVelocity(3000);
      Robot.flywheel.setFlywheelState(FlywheelState.SPINNINGUP);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    horizontalOffset.add(Hardware.limelight.getHorizontalAngle());
    verticalOffset.add(Hardware.limelight.getHorizontalAngle());

    Robot.drivetrain.alignToTarget(horizontalOffset.getAverage());
    double avgDistance = Hardware.limelight.getDistanceFromTarget(verticalOffset.getAverage());

    if(RPM == 0 ){
      Robot.flywheel.updateTargetVelocity(Hardware.limelight.getRPMFromDistance(avgDistance));
    }

    if ((Hardware.limelight.hasTarget() && horizontalOffset.allWithinError(0, 2.6) && Robot.flywheel.getFlywheelState() == FlywheelState.ATSPEED) || Robot.oi.getShooterOverrideButton()) {
      Robot.hopper.runHopper(0.54, .54);
      Robot.led.setColor(LEDColor.PURPLE);
    }else{
      if(!Robot.hopper.getTopBreakbeam()){
          Robot.hopper.runHopper(.4, .4);
      }
      Robot.led.setColor(LEDColor.YELLOW);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.stop();
    Robot.hopper.stopMotors();
    Robot.flywheel.setFlywheelState(FlywheelState.OFF);
    Hardware.limelight.setLED(LED_MODE.OFF);
   
   if(Robot.getRobotState() == RobotState.TELEOP)
    Robot.led.setColor(LEDColor.BLUE);
   else
    Robot.led.setColor(LEDColor.RAINBOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Robot.getRobotState() == RobotState.TELEOP) {
      return !Robot.oi.getAlignButton() && !Robot.oi.getControlPanelShot();
    }

    return false;

  }
}

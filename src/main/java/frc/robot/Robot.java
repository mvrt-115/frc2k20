/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.AutonRoutine;
import frc.robot.commands.AutonRoutine2;
import frc.robot.commands.AutonRoutine3;
import frc.robot.commands.BasicAuto;
import frc.robot.commands.RamseteCommand;
import frc.robot.commands.RendezvousAuton2;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.LED_MODE;
import frc.robot.util.Limelight.PIPELINE_STATE;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Climber.ElevatorState;
import frc.robot.subsystems.Flywheel.FlywheelState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.LEDStrip.LEDColor;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static Flywheel flywheel;
  public static Drivetrain drivetrain;
  public static Intake intake;
  public static Hopper hopper;
  public static Climber climber;
  public static LEDStrip led;
  public static OI oi;
  
  private SendableChooser<Command> autonSelector;

  public enum RobotState{
    DISABLED, TELEOP, AUTON
  };

  private static RobotState currState = RobotState.DISABLED;
  private double startDisabledTime;
  public double desiredRPM;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    flywheel =  new Flywheel();
    drivetrain = new Drivetrain();
    intake = new Intake();
    hopper = new Hopper();
    climber = new Climber();
    led = new LEDStrip();
    Hardware.limelight = new Limelight();
    oi = new OI();

    setRobotState(RobotState.DISABLED);

    Hardware.limelight.setPipeline(PIPELINE_STATE.VISION_WIDE);
    Hardware.limelight.setLED(LED_MODE.OFF);
    flywheel.setFlywheelState(FlywheelState.OFF);
    CameraServer.getInstance().startAutomaticCapture();

    autonSelector = new SendableChooser<>();

    autonSelector.setDefaultOption("Basic Shoot", new AutonRoutine3());
    autonSelector.addOption("Trench Run", new AutonRoutine());
    autonSelector.addOption("Rendezvous Run", new AutonRoutine2());
    autonSelector.addOption("Rendezvous Run Small", new RendezvousAuton2());
    autonSelector.addOption("Shoot then Back", new BasicAuto());
    SmartDashboard.putData(autonSelector);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  //  desiredRPM =  SmartDashboard.getNumber("Desired RPM", 0);
   // flywheel.updateTargetVelocity(desiredRPM);
    flywheel.log();
    drivetrain.log();
    hopper.log();
    intake.log();
    climber.log();
    
    CommandScheduler.getInstance().run();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    setRobotState(RobotState.DISABLED);
    Hardware.limelight.setLED(LED_MODE.OFF);
    led.setColor(LEDColor.MVRT); 
    drivetrain.configNeutralMode(NeutralMode.Brake);
    startDisabledTime = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledPeriodic() {
    Hardware.limelight.setLED(LED_MODE.OFF);
    if(Timer.getFPGATimestamp() - startDisabledTime > 2)
      drivetrain.configNeutralMode(NeutralMode.Coast);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    setRobotState(RobotState.AUTON);

    intake.resetPivotEncoder();
    intake.setIntakeState(IntakeState.STOWED);
    intake.setDefaultLimitSwitchStart();
    
    climber.setElevatorState(ElevatorState.ZEROED);
    climber.resetEncoder();
    
    flywheel.setFlywheelState(FlywheelState.OFF);

    drivetrain.resetOdometry();
    drivetrain.configNeutralMode(NeutralMode.Brake, NeutralMode.Brake);

    led.setColor(LEDColor.RAINBOW);

    hopper.setBalls(3);
    
    m_autonomousCommand = autonSelector.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    led.setColor(LEDColor.BLUE);

        // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    setRobotState(RobotState.TELEOP);
    climber.setElevatorState(ElevatorState.ZEROED);
    drivetrain.configNeutralMode(NeutralMode.Coast, NeutralMode.Coast);

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void setRobotState(final RobotState newState) {
    currState = newState;
  }

  public static RobotState getRobotState(){
    return currState;
  }
  
  public static Command generatePath(Trajectory trajectory){
    
    var autoCommand = new RamseteCommand(
      trajectory,
      drivetrain::getPose,
      drivetrain.getRamseteController(),
      drivetrain.getFeedForward(),
      drivetrain.getDriveKinematics(),
      drivetrain::getWheelSpeeds,
      drivetrain.getLeftDriveController(),
      drivetrain.getRightDriveController(),
      drivetrain::setOutputVolts,
      drivetrain
    );

  
    return autoCommand;
  }

}

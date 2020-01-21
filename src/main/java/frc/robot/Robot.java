/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  public static Drivetrain drivetrain;
  public static OI oi;
  private Command m_autonomousCommand;

  public enum RobotState{
    DISABLED, TELEOP, AUTON
  };

  private RobotState currState;
  RamseteCommand autoCommand;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    drivetrain = new Drivetrain();
    Hardware.limelight = new Limelight();
    oi = new OI();
    setRobotState(RobotState.DISABLED);
 

    //read values periodically
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    drivetrain.log();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    setRobotState(RobotState.DISABLED);
  }

  @Override
  public void disabledPeriodic() 
  {

  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() 
  {

    
    drivetrain.resetOdometry();
    m_autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.schedule();
    }
    setRobotState(RobotState.AUTON);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() 
  {
  
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.cancel();
    }
    setRobotState(RobotState.TELEOP);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() 
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {
    
  }

  public void setRobotState(final RobotState newState) {
    currState = newState;
  }

  public Command getAutonomousCommand() {
   
    var waypoints = Arrays.asList(
      new Pose2d(0,0, Rotation2d.fromDegrees(0)),
      new Pose2d(2,0, Rotation2d.fromDegrees(0))
      
    );

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, drivetrain.getTrajectoryConfig());

     autoCommand = new RamseteCommand(
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

    

    return autoCommand.andThen(() -> drivetrain.setLeftRightMotorOutputs(0, 0));
  }
}

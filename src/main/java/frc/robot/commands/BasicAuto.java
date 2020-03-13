/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class BasicAuto extends SequentialCommandGroup {
  /**
   * Right Side Trench Auton
   */
  public BasicAuto() {
    
    addCommands(
      
      new WaitCommand(6),
      new AutoShoot(6800).withTimeout(6.5),
      getTrajectory1());
  }

  public Command getTrajectory1(){
    Robot.drivetrain.invertPathDirection(true);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(0, 0, new Rotation2d()),  
      new Pose2d(-1,0, new Rotation2d().fromDegrees(0))

    ), Robot.drivetrain.getTrajectoryConfig());

    return Robot.generatePath(traj1);
  }
}
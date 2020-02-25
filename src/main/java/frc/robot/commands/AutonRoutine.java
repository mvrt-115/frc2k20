/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonRoutine extends SequentialCommandGroup {
  /**
   * Right Side Trench Auton
   */
  public AutonRoutine() {
    
    addCommands(
        
    
      new AutoShoot(6800).withTimeout(4),
        new ParallelRaceGroup(
          new ParallelCommandGroup(
            getTrajectory1(),
            new IntakeCommand().withTimeout(5.5)
          ),
          new AutoHopper().withTimeout(10)
        ),
        new ParallelRaceGroup(
          new SequentialCommandGroup(
            getTrajectory2(),
            new AutoShoot(6200).withTimeout(5)
          ),  
          new IntakeCommand().withTimeout(5)
        )
    );
  }

  public Command getTrajectory2(){
    
    Robot.drivetrain.invertPathDirection(false);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(-4, -1.55, new Rotation2d()),  
      new Pose2d(-1.25,0, new Rotation2d().fromDegrees(8))

    ), Robot.drivetrain.getTrajectoryConfig());

    
    // String trajectoryJSON = "paths/Right-Path2.wpilib.json";
    // Path trajectoryPath;
    // try {
    //   trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //   TrajectoryUtil.toPathweaverJson(traj1, trajectoryPath);
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
 //   }
 
    return Robot.generatePath(traj1);
  }

  public Command getTrajectory1(){
    Robot.drivetrain.invertPathDirection(true);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(0, 0, new Rotation2d()),  
      new Pose2d(-2.2,-1.55, new Rotation2d().fromDegrees(8)),
      new Pose2d(-4, -1.55, new Rotation2d())

    ), Robot.drivetrain.getTrajectoryConfig());

    return Robot.generatePath(traj1);
  }
}
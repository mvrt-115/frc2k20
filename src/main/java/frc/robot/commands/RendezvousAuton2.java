
package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RendezvousAuton2 extends SequentialCommandGroup {
  /**
   * Rendezvous Auton Routine
   */
  public RendezvousAuton2() {

    addCommands(
        new ParallelRaceGroup(    
            new SequentialCommandGroup(
              new ParallelCommandGroup(
                getTrajectory1(),
                new IntakeCommand().withTimeout(3)
              ),
              new ParallelRaceGroup(
                getTrajectory2(),
                new IntakeCommand()
            )
          ),
          new AutoHopper().withTimeout(15)
        ),
        new AutoShoot(6600).withTimeout(6)
      );
  }

  public Command getTrajectory1(){
    Robot.drivetrain.invertPathDirection(true);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(),
      new Pose2d(-2.05, -.2, new Rotation2d(.4,.161))
     // new Pose2d(-2, -.2, new Rotation2d(1.52, .62))
    ), Robot.drivetrain.getTrajectoryConfig());

    
    return Robot.generatePath(traj1);
  }

  public Command getTrajectory2(){
    Robot.drivetrain.invertPathDirection(false);

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(List.of(
      
    new Pose2d(-2.05, -.2, new Rotation2d(.4,.161)),
      new Pose2d(-.3, -.8, new Rotation2d(.569,-.223))

    ), Robot.drivetrain.getTrajectoryConfigSlow());

    return Robot.generatePath(traj1);
  }
}

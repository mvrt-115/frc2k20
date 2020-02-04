/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonRoutine extends SequentialCommandGroup {
  /**
   * Creates a new AutonRoutine.
   */
  public AutonRoutine(Command trajectory1, Command trajectory2) {
    addCommands(
      new AutoShoot(8, 3000),
      new ParallelCommandGroup(trajectory1,
        new SequentialCommandGroup(
          new WaitCommand(3),
          new IntakeCommand(5)
        )
      ),
      new AutoShoot(7, 6000)
    );
  }
}
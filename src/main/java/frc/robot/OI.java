/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;

/**
 * Add your docs here.
 */
public class OI {

    private Joystick joystick;
    private JoystickButton intakeButton;
    private JoystickButton outtakeButton;

    public OI(){
        joystick = new Joystick(0);
        intakeButton = new JoystickButton(joystick, 1);
        outtakeButton = new JoystickButton(joystick, 2);

        intakeButton.whenPressed(new IntakeCommand());
        outtakeButton.whenPressed(new OuttakeCommand());
    }

    public boolean isIntakePressed(){
        return intakeButton.get();
    }

    public boolean isOuttakePressed(){
        return outtakeButton.get();
    }

}

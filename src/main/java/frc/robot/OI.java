/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbAndLevelCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.commands.ZeroElevator;

/**
 * Add your docs here.
 */
public class OI {

    private Joystick operatorJoystick;
    private JoystickButton raiseElevatorButton;
    private JoystickButton climbButton;
    private JoystickButton zeroElevatorButton;

    public OI(){
        operatorJoystick = new Joystick(1);

        raiseElevatorButton = new JoystickButton(operatorJoystick, 1);
        climbButton = new JoystickButton(operatorJoystick, 2);
        zeroElevatorButton = new JoystickButton(operatorJoystick, 3);

        raiseElevatorButton.whenPressed(new RaiseElevatorCommand());
        climbButton.whenPressed(new ClimbAndLevelCommand());
        zeroElevatorButton.whenActive(new ZeroElevator());
    }
}

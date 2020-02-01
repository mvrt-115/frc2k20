/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoHopper;

/**
 * Add your docs here.
 */
public class OI {

    Joystick operatorJoystick;
    JoystickButton hopperButton;

    public OI() {
        operatorJoystick = new Joystick(1);

        hopperButton = new JoystickButton(operatorJoystick, 2);


        Robot.hopper.setDefaultCommand(new AutoHopper());

    }

    public boolean getHopperButton()
    {
        return hopperButton.get();
    }
}

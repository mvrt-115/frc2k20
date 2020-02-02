/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithColor;

/**
 * Add your docs here.
 */
public class OI 
{
    public Joystick joystick;
    public JoystickButton goToColor;
   
    public OI()
    {
        joystick = new Joystick(0);
        goToColor = new JoystickButton(joystick, 1);

        goToColor.whenHeld(new DriveWithColor());
    }

    public double getX()
    { 
        return joystick.getRawAxis(4);
    }

    public double getY()
    { 
        return joystick.getRawAxis(1);
    }

    public boolean getColorButton()
    {
        return goToColor.get();
    }
}

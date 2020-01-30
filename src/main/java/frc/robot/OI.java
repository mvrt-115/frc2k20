/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SetFlywheelRPM;

/**
 * Add your docs here.
 */
public class OI {

    private Joystick driverJoystick;
    private JoystickButton shootBall;
    private JoystickButton stopFlywheel;

    public OI(){
        driverJoystick = new Joystick(0);
        shootBall = new JoystickButton(driverJoystick, 1);
        stopFlywheel = new JoystickButton(driverJoystick, 2);

        shootBall.whenPressed(new SetFlywheelRPM(6750));
        stopFlywheel.whenPressed(new SetFlywheelRPM(0));
    }


}

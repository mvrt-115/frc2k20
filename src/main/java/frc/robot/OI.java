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
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class OI {

    private Joystick driverJoystick;
    private JoystickButton shootBall;
    private JoystickButton stopFlywheel;

    private Joystick operatorJoystick;
    private JoystickButton align;
    private JoystickButton quickTurnButton;

    public OI(){
        driverJoystick = new Joystick(0);
        shootBall = new JoystickButton(driverJoystick, 1);
        stopFlywheel = new JoystickButton(driverJoystick, 2);

      
        operatorJoystick = new Joystick(1);
        align = new JoystickButton(driverJoystick, 8);
        quickTurnButton = new JoystickButton(driverJoystick, 5);
        align.whenPressed(new AutoAlign());

        Robot.drivetrain.setDefaultCommand(new DriveWithJoystick());

        shootBall.whenPressed(new SetFlywheelRPM(6750));
        stopFlywheel.whenPressed(new SetFlywheelRPM(0));
    }

    public double getWheel()
    {
        double wheel = driverJoystick.getRawAxis(0);
      //  if(Math.abs(wheel) >= wheelDeadband) return (wheel - wheelDeadband * Math.abs(wheel) / wheel) / (1 - wheelDeadband);
        return wheel;
    }

    public double getThrottle()
    {
       double throttle = driverJoystick.getRawAxis(5);
   //     if(Math.abs(throttle) >= 0.1) return (throttle - throttleDeadband * Math.abs(throttle) / throttle) / (1 - throttleDeadband);
        return -throttle;
    }

    public boolean getAlign() {
        return align.get();
    }

    public boolean getQuickTurn(){
        return quickTurnButton.get();
    }
}


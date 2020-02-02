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
import frc.robot.commands.AutoHopper;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ClimbAndLevelCommand;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualHopper;
import frc.robot.commands.RaiseElevatorCommand;

/**
 * Add your docs here.
 */
public class OI {

    private Joystick driverJoystick;
    private JoystickButton shootBall;
    private JoystickButton stopFlywheel;
    private JoystickButton alignButton;
    private JoystickButton quickTurnButton;


    private Joystick operatorJoystick;
    private JoystickButton intakeButton;
    private JoystickButton hopperButton;
    private JoystickButton raiseElevatorButton;
    private JoystickButton climbButton;

    public OI(){
        driverJoystick = new Joystick(0);
        operatorJoystick = new Joystick(1);

        shootBall = new JoystickButton(driverJoystick, 1);
        stopFlywheel = new JoystickButton(driverJoystick, 2);
        alignButton = new JoystickButton(driverJoystick, 8);
        quickTurnButton = new JoystickButton(driverJoystick, 5);

        intakeButton = new JoystickButton(operatorJoystick, 1);
        hopperButton = new JoystickButton(operatorJoystick, 2);
        raiseElevatorButton = new JoystickButton(operatorJoystick, 1);
        climbButton = new JoystickButton(operatorJoystick, 2);

        
        raiseElevatorButton.whenPressed(new RaiseElevatorCommand());
        climbButton.whenPressed(new ClimbAndLevelCommand());
        intakeButton.whenPressed(new IntakeCommand());
        shootBall.whenPressed(new SetFlywheelRPM(8320));
        stopFlywheel.whenPressed(new SetFlywheelRPM(0));
        alignButton.whenPressed(new AutoShoot(0));
        hopperButton.whenPressed(new ManualHopper());

        Robot.hopper.setDefaultCommand(new AutoHopper());
        Robot.drivetrain.setDefaultCommand(new DriveWithJoystick());
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

    public boolean getAlignButton() {
        return alignButton.get();
    }

    public boolean getQuickTurn(){
        return quickTurnButton.get();
    }

    public boolean isIntakeButtonPressed(){
        return intakeButton.get();
    }

    public boolean getHopperButton()
    {
        return hopperButton.get();
    }
}

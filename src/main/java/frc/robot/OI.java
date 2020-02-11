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
import frc.robot.util.JoystickTrigger;
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
    private JoystickTrigger hopperForwardButton;
    private JoystickTrigger hopperReverseButton;
    private JoystickButton raiseElevatorButton;
    private JoystickButton climbButton;

    public OI(){
        driverJoystick = new Joystick(0);
        operatorJoystick = new Joystick(1);

        shootBall = new JoystickButton(driverJoystick, 9);
        stopFlywheel = new JoystickButton(driverJoystick, 10);
        alignButton = new JoystickButton(driverJoystick, 6);
        quickTurnButton = new JoystickButton(driverJoystick, 5); 
        intakeButton = new JoystickButton(driverJoystick, 8);

        hopperForwardButton = new JoystickTrigger(operatorJoystick, 4);
        hopperReverseButton = new JoystickTrigger(operatorJoystick, 5);
        raiseElevatorButton = new JoystickButton(operatorJoystick, 4);
        climbButton = new JoystickButton(operatorJoystick, 5);
        
        raiseElevatorButton.whenPressed(new RaiseElevatorCommand());
        climbButton.whenPressed(new ClimbAndLevelCommand());
        intakeButton.whenPressed(new IntakeCommand(0));
        shootBall.whenPressed(new SetFlywheelRPM(3000));
        stopFlywheel.whenPressed(new SetFlywheelRPM(0));
        alignButton.whenPressed(new AutoShoot(0, 0));
        hopperForwardButton.whenActive(new ManualHopper(0.3, 0.3));
        hopperReverseButton.whenActive(new ManualHopper(-0.3, -0.3));

        Robot.hopper.setDefaultCommand(new AutoHopper());
        Robot.drivetrain.setDefaultCommand(new DriveWithJoystick());
    }

    public double getWheel()
    {
        double wheel = driverJoystick.getRawAxis(0);
        return wheel;
    }

    public double getThrottle()
    {
       double throttle = driverJoystick.getRawAxis(5);
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
        return hopperForwardButton.get() || hopperReverseButton.get();
    }
}

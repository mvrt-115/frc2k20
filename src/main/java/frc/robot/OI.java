/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SetFlywheelRPM;
import frc.robot.util.JoystickTrigger;
import frc.robot.commands.AutoHopper;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ElevatorSetpoint;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualBallOverride;
import frc.robot.commands.ManualHopper;
import frc.robot.commands.OuttakeCommand;

/**
 * Add your docs here.
 */
public class OI 
{
    private Joystick driverJoystick;

    private JoystickButton intake;
    private JoystickButton outtake;

    private JoystickButton shootBall;
    private JoystickButton stopFlywheel;
    private JoystickButton alignButton;
    private JoystickButton quickTurnButton;

    private JoystickButton test;
    private Joystick operatorJoystick;
    private JoystickButton intakeButton;
    private POVButton hopperForwardButton;
    private POVButton hopperReverseButton;
    private POVButton hopperIncButton;
    private POVButton hopperDecButton;
    private JoystickButton raiseElevatorButton;
    private JoystickButton climbButton;
    private JoystickButton zeroButton;

    public OI(){
        driverJoystick = new Joystick(0);
        operatorJoystick = new Joystick(1);

        intake = new JoystickButton(driverJoystick, 1);
        outtake = new JoystickButton(driverJoystick, 2);
        shootBall = new JoystickButton(operatorJoystick, 1); // 9
        stopFlywheel = new JoystickButton(operatorJoystick, 2); // 10
        quickTurnButton = new JoystickButton(driverJoystick, 5); 
        alignButton = new JoystickButton(driverJoystick, 7);
        intakeButton = new JoystickButton(driverJoystick, 8); // 8

        hopperForwardButton = new POVButton(operatorJoystick, 0);
        hopperReverseButton = new POVButton(operatorJoystick, 180);
        hopperIncButton = new POVButton(operatorJoystick, 90);
        hopperDecButton = new POVButton(operatorJoystick, 270);
        raiseElevatorButton = new JoystickButton(operatorJoystick, 7); // 8
        climbButton = new JoystickButton(driverJoystick, 3);
        zeroButton = new JoystickButton(operatorJoystick, 8);

        test = new JoystickButton(driverJoystick, 3);
        
        raiseElevatorButton.whenPressed(new ElevatorSetpoint(Constants.kClimbHeight));
        zeroButton.whenPressed(new ElevatorSetpoint(Constants.kElevatorZero));
        climbButton.whenPressed(new ClimbCommand());
        intakeButton.whenActive(new IntakeCommand());
        //shootBall.whenPressed(new SetFlywheelRPM(4500));
        //stopFlywheel.whenPressed(new SetFlywheelRPM(0));
        alignButton.whenActive(new AutoShoot(0));
        hopperForwardButton.whenActive(new ManualHopper(0.3, 0.3));
        hopperReverseButton.whenActive(new ManualHopper(-0.3, -0.3));
        hopperIncButton.whenPressed(new ManualBallOverride(1));
        hopperDecButton.whenPressed(new ManualBallOverride(-1));
        test.whenPressed(new OuttakeCommand());
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

    public boolean getTest(){
        return !test.get();
    }
}

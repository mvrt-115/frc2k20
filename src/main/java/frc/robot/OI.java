/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.util.DualshockController;
import frc.robot.util.JoystickTrigger;
import frc.robot.util.DualshockController.Button;
import frc.robot.commands.AutoHopper;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ChangeServoPosition;
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
public class OI {

    private DualshockController driverJoystick;
    private JoystickButton alignButton;
    private JoystickButton quickTurnButton;
    private JoystickButton intakeButton;
    private JoystickButton controlPanelShot;
    private JoystickButton shooterOverrideButton;
    private JoystickButton intakeReverseButton;

    public Joystick operatorJoystick;
    private POVButton hopperForwardButton;
    private POVButton hopperReverseButton;
    private POVButton hopperIncButton;
    private POVButton hopperDecButton;
    private JoystickButton raiseElevatorButton;
    private JoystickButton climbButton;
    private JoystickButton zeroButton;
    private JoystickTrigger servoChange;

    public OI(){
        driverJoystick = new DualshockController(0);
        operatorJoystick = new Joystick(1);

        quickTurnButton = new JoystickButton(driverJoystick, Button.kBumperLeft.value);
        alignButton = new JoystickButton(driverJoystick, Button.kTriggerLeft.value);
        intakeButton = new JoystickButton(driverJoystick, Button.kTriggerRight.value);
        shooterOverrideButton = new JoystickButton(driverJoystick, Button.kSquare.value);
        intakeReverseButton = new JoystickButton(driverJoystick, Button.kCircle.value);
        controlPanelShot = new JoystickButton(driverJoystick,Button.kOption.value);

        
        hopperForwardButton = new POVButton(operatorJoystick, 0);
        hopperReverseButton = new POVButton(operatorJoystick, 180);
        hopperIncButton = new POVButton(operatorJoystick, 90);
        hopperDecButton = new POVButton(operatorJoystick, 270);
        raiseElevatorButton = new JoystickButton(operatorJoystick, 4);
        climbButton = new JoystickButton(operatorJoystick, 1);
        zeroButton = new JoystickButton(operatorJoystick, 2);
        servoChange = new JoystickTrigger(operatorJoystick, 3);
        
  /*      driverJoystick = new Joystick(0);
        operatorJoystick = new Joystick(1);

        quickTurnButton = new JoystickButton(driverJoystick, 5); 
        alignButton = new JoystickButton(driverJoystick, 7);        //2
        intakeButton = new JoystickButton(driverJoystick, 8);       //3 
        shooterOverrideButton = new JoystickButton(driverJoystick, 1);
        controlPanelShot = new JoystickButton(driverJoystick, 6);
        intakeReverseButton = new JoystickButton(driverJoystick, 3);

*/
        raiseElevatorButton.whenPressed(new ElevatorSetpoint(Constants.kClimbHeight));
        zeroButton.whenPressed(new ElevatorSetpoint(Constants.kElevatorZero));
        climbButton.whenPressed(new ClimbCommand());
        intakeButton.whenActive(new IntakeCommand());
        alignButton.whenActive(new AutoShoot(0));
        controlPanelShot.whenPressed(new AutoShoot(7500));
        hopperForwardButton.whenActive(new ManualHopper(0.65, 0.65));
        hopperReverseButton.whenActive(new ManualHopper(-0.85, -0.65));
        hopperIncButton.whenPressed(new ManualBallOverride(1));
        hopperDecButton.whenPressed(new ManualBallOverride(-1));
        intakeReverseButton.whenPressed(new OuttakeCommand());
        Robot.hopper.setDefaultCommand(new AutoHopper());
        Robot.drivetrain.setDefaultCommand(new DriveWithJoystick());
        servoChange.whenActive(new ChangeServoPosition());
        
    }

    public boolean getControlPanelShot(){
        return controlPanelShot.get();
    }

    public double getWheel()
    {
        double wheel = driverJoystick.getX(Hand.kLeft);
        return wheel;
    }

    public double getThrottle()
    {
       double throttle = driverJoystick.getY(Hand.kRight);
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

    public boolean getIntakeReverse(){
        return !intakeReverseButton.get();
    }

    public boolean getShooterOverrideButton(){
        return shooterOverrideButton.get();
    }
}

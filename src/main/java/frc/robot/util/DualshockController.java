/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Add your docs here.
 */
public class DualshockController extends GenericHID {

    public enum Button {
        kBumperLeft(5), kBumperRight(6), kStickLeft(11), kStickRight(12), kX(2), kSquare(1), kTriangle(4), kCircle(3),
        kOption(10), kShare(9), kMiddle(14), kTriggerLeft(7), kTriggerRight(8), kPlaystation(13);

        @SuppressWarnings({ "MemberName", "PMD.SingularField" })
        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    /**
     * Represents an axis on an XboxController.
     */
    public enum Axis {
        kLeftX(0), kRightX(2), kLeftY(1), kRightY(5), kLeftTrigger(3), kRightTrigger(4);

        @SuppressWarnings({ "MemberName", "PMD.SingularField" })
        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public DualshockController(int port) {
        super(port);
        // TODO Auto-generated constructor stub
        HAL.report(tResourceType.kResourceType_Joystick, port + 1);
    }

    @Override
    public double getX(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawAxis(Axis.kLeftX.value);
        } else {
            return getRawAxis(Axis.kRightX.value);
        }
    }

    /**
     * Get the Y axis value of the controller.
     *
     * @param hand Side of controller whose value should be returned.
     * @return The Y axis value of the controller.
     */
    @Override
    public double getY(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawAxis(Axis.kLeftY.value);
        } else {
            return getRawAxis(Axis.kRightY.value);
        }
    }

    /**
     * Get the trigger axis value of the controller.
     *
     * @param hand Side of controller whose value should be returned.
     * @return The trigger axis value of the controller.
     */
    public double getTriggerAxis(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawAxis(Axis.kLeftTrigger.value);
        } else {
            return getRawAxis(Axis.kRightTrigger.value);
        }
    }

    public boolean getTriggerButton(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawButton(Axis.kLeftTrigger.value);
        } else {
            return getRawButton(Axis.kRightTrigger.value);
        }
    }

    /**
     * Read the value of the bumper button on the controller.
     *
     * @param hand Side of controller whose value should be returned.
     * @return The state of the button.
     */
    public boolean getBumper(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawButton(Button.kBumperLeft.value);
        } else {
            return getRawButton(Button.kBumperRight.value);
        }
    }

    /**
     * Whether the bumper was pressed since the last check.
     *
     * @param hand Side of controller whose value should be returned.
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBumperPressed(Hand hand) {
        if (hand == Hand.kLeft) {
            return getRawButtonPressed(Button.kBumperLeft.value);
        } else {
            return getRawButtonPressed(Button.kBumperRight.value);
        }
    }

    /**
     * Whether the bumper was released since the last check.
     *
     * @param hand Side of controller whose value should be returned.
     * @return Whether the button was released since the last check.
     */
    public boolean getBumperReleased(Hand hand) {
        if (hand == Hand.kLeft) {
            return getRawButtonReleased(Button.kBumperLeft.value);
        } else {
            return getRawButtonReleased(Button.kBumperRight.value);
        }

    }

    /**
     * Read the value of the stick button on the controller.
     *
     * @param hand Side of controller whose value should be returned.
     * @return The state of the button.
     */
    public boolean getStickButton(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawButton(Button.kStickLeft.value);
        } else {
            return getRawButton(Button.kStickRight.value);
        }
    }

    /**
     * Whether the stick button was pressed since the last check.
     *
     * @param hand Side of controller whose value should be returned.
     * @return Whether the button was pressed since the last check.
     */
    public boolean getStickButtonPressed(Hand hand) {
        if (hand == Hand.kLeft) {
            return getRawButtonPressed(Button.kStickLeft.value);
        } else {
            return getRawButtonPressed(Button.kStickRight.value);
        }
    }

    /**
     * Whether the stick button was released since the last check.
     *
     * @param hand Side of controller whose value should be returned.
     * @return Whether the button was released since the last check.
     */
    public boolean getStickButtonReleased(Hand hand) {
        if (hand == Hand.kLeft) {
            return getRawButtonReleased(Button.kStickLeft.value);
        } else {
            return getRawButtonReleased(Button.kStickRight.value);
        }
    }

    /**
     * Read the value of the A button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getXButton() {
        return getRawButton(Button.kX.value);
    }

    /**
     * Read the value of the B button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getTriangleButton() {
        return getRawButton(Button.kTriangle.value);
    }


    /**
     * Read the value of the X button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getSquareButton() {
        return getRawButton(Button.kSquare.value);
    }


    /**
     * Read the value of the Y button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getCircleButton() {
        return getRawButton(Button.kCircle.value);
    }


    /**
     * Read the value of the back button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getShareButton() {
        return getRawButton(Button.kShare.value);
    }


    /**
     * Read the value of the start button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getOptionButton() {
        return getRawButton(Button.kOption.value);
    }

}

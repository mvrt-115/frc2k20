/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Add your docs here.
 */
public class JoystickTrigger extends Trigger{

    private  Joystick joystick;
    private int axis;

    public JoystickTrigger(Joystick joystick, int axis) {
        super();
        this.joystick = joystick;
        this.axis = axis;
    }


    public boolean get() {
        double value = joystick.getRawAxis(axis); 
        
        if(value > 0.3) 
            return true;
        else
         return false;
    }


}
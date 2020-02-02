/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;

/**
 * Add your docs here.
 */
public class Hardware 
{
    public static TalonSRX left;
    public static TalonSRX right;
    public static TalonSRX top;

    public static I2C.Port i2cPort;
    /**
    * A Rev Color Sensor V3 object is constructed with an I2C port as a 
    * parameter. The device will be automatically initialized with default 
    * parameters.
    */
    public static ColorSensorV3 m_colorSensor;

    /**
    * A Rev Color Match object is used to register and detect known colors. This can 
    * be calibrated ahead of time or during operation.
    * 
    * This object uses a simple euclidian distance to estimate the closest match
    * with given confidence range.
    */
    public static ColorMatch m_colorMatcher;
}

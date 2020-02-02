/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Robot;

public class ColorSensor extends SubsystemBase {
  /**
   * Creates a new SenseColor.
   */
  Color tarColor;
  Color currColor;
  Color prevColor;
  boolean same = false;
  int count = 0;

  public ColorSensor() 
  {
    Hardware.top = new TalonSRX(17);
    Hardware.i2cPort = I2C.Port.kOnboard;
    Hardware.m_colorSensor = new ColorSensorV3(Hardware.i2cPort);
    Hardware.m_colorMatcher = new ColorMatch();

    tarColor = getColor();
    currColor = Hardware.m_colorSensor.getColor();
  }

  public void rotate()
  {
    Hardware.top.set(ControlMode.PercentOutput, getSpeed());
  }

  public void stop()
  {
    Hardware.top.set(ControlMode.PercentOutput, 0);
  }

  public double getSpeed()
  {
    Color detectedColor = Hardware.m_colorSensor.getColor();
    ColorMatchResult match = Hardware.m_colorMatcher.matchClosestColor(detectedColor);
    currColor = match.color;

    if(currColor != tarColor)
    {
      same = false;
      return 0.1;
    }
    else if(currColor == tarColor && count != 4 && !same)
    {
      count++;
      same = true;
    }

    if(currColor == tarColor && count != 4) return 0.1;
    
    if(count == 4) return 0;

    return 0;
  }
  
  public Color getColor()
  {
      String color = DriverStation.getInstance().getGameSpecificMessage();
      if(color != null && color.length() > 0)
      {
          switch(color.charAt(0))
          {
              case 'B': return Constants.BLUE;
              case 'G': return Constants.GREEN;
              case 'Y': return Constants.YELLOW;
              case 'R': return Constants.RED;
              default : break;
          }
      }
      return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;

import java.awt.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Climber.ElevatorState;
 
public class LEDStrip extends SubsystemBase
{
  private static Color[] colors;
  public static AddressableLED m_led;
  public static AddressableLEDBuffer m_ledBuffer;
  public static int m_rainbowFirstPixelHue;
  public static Color purple, gold;
  public static long lastTime;
  public static int lastBalls;
  public static int redValue;
  public static int changeValue;
 
  /**
   * Creates a new LEDStrip.
   */
  public LEDStrip()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_led = new AddressableLED(7);
 
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    m_rainbowFirstPixelHue = 0;
 
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    purple = new Color(84, 0, 84);
    gold = new Color(240, 100, 0);

    colors = new Color[m_ledBuffer.getLength()];

    lastTime = 0;
    lastBalls = 0;
    redValue = 150;
    changeValue = -10;

    MVRT();
  }

  public static void MVRT()
  {
    for(int i = 0; i < m_ledBuffer.getLength(); i++)
    {
      if(i % 10 >= 5) colors[i] = gold;
      else colors[i] = purple;
      m_ledBuffer.setRGB(i, colors[i].getRed(), colors[i].getGreen(), colors[i].getBlue());
    }
  }

  public static void rotate()
  {
    Color temp = colors[0];

    for(int i = 0; i < m_ledBuffer.getLength() - 1; i++)
    {
      colors[i] = colors[i + 1];
      m_ledBuffer.setRGB(i, colors[i].getRed(), colors[i].getGreen(), colors[i].getBlue());
    }

    colors[colors.length - 1] = temp;
    m_ledBuffer.setRGB(colors.length - 1, colors[colors.length - 1].getRed(), colors[colors.length - 1].getGreen(), colors[colors.length - 1].getBlue());
    m_led.setData(m_ledBuffer);
  }
 
  public static void red()
  {
    for(var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, redValue, 0, 0);
    }
   
    if(redValue == 10 || redValue == 150) changeValue *= -1;
    redValue += changeValue;
    m_led.setData(m_ledBuffer);
  }
 
  public static void green()
  {
    for(var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, 0, 255, 0);
    }
   
    m_led.setData(m_ledBuffer);
  }
 
  public static void blue()
  {
    for(var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
   
    m_led.setData(m_ledBuffer);
  }
 
  public static void yellow()
  {
    for(var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, 54, 55, 83);
    }
   
    m_led.setData(m_ledBuffer);
  }
 
  public static void updateHopperStrip()
  {
    int balls = Robot.hopper.getBalls();
    if(balls <= 5 && balls > 0)
    {
      for(int i = 0; i < m_ledBuffer.getLength() * balls / 5; i++)
      {
        m_ledBuffer.setRGB(i, 255, 0, 0);
      }

      for(int i = m_ledBuffer.getLength() * balls / 5; i < m_ledBuffer.getLength(); i++)
      {
        m_ledBuffer.setRGB(i, 0, 255, 0);
      }
    }
  }

  public static void rainbow() 
  {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
 
  @Override
  public void periodic() 
  {
    if(RobotState.isDisabled()) red();
    else if(Robot.climber.getElevatorState() != ElevatorState.ZEROED) rainbow();
    else if(Robot.hopper.getBalls() == 0)
    {
      if(lastBalls != 0)
      {
        MVRT();
        lastBalls = 0;
      }
      if(System.currentTimeMillis() - lastTime >= 50) 
      {
        rotate();
        lastTime = System.currentTimeMillis();
      }
    }
    else 
    {
      updateHopperStrip();
      lastBalls = Robot.hopper.getBalls();
    }

    m_led.setData(m_ledBuffer);
  }
}
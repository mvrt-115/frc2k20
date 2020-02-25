/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.awt.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
 
public class LEDStrip extends SubsystemBase
{
  private Color[] colors;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private Color purple, gold;
  private long lastTime;
  private int redValue;
  private int changeValue;

  private LEDColor currColor;

  public enum LEDColor {
    RAINBOW, MVRT, PURPLE, BLUE, YELLOW, COUNTER, RED
  } 

  /**
   * Creates a new LEDStrip.
   */
  public LEDStrip()
  {
    m_led = new AddressableLED(1);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(40);
    m_led.setLength(m_ledBuffer.getLength());
    m_rainbowFirstPixelHue = 0;
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    purple = new Color(84, 0, 84);
    gold = new Color(240, 100, 0);

    colors = new Color[m_ledBuffer.getLength()];

    lastTime = 0;
    redValue = 150;
    changeValue = -10;
  }


  public void setColor(LEDColor _desiredColor){
    currColor = _desiredColor;

    if(_desiredColor == LEDColor.MVRT){
      MVRT();
    }
  }

  public LEDColor getColor(){
    return currColor;
  }

  public void MVRT()
  {
    for(int i = 0; i < m_ledBuffer.getLength(); i++)
    {
      if(i % 10 >= 5) colors[i] = gold;
      else colors[i] = purple;
      m_ledBuffer.setRGB(i, colors[i].getRed(), colors[i].getGreen(), colors[i].getBlue());
    }
  }

  public void rotate()
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
 
  public void red()
  {
    for(var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, redValue, 0, 0);
    }
   
    if(redValue == 10 || redValue == 150) changeValue *= -1;
    redValue += changeValue;
    m_led.setData(m_ledBuffer);

  }
 
  public void purple()
  {
    for(var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, 84, 0, 84);
    }
   
    m_led.setData(m_ledBuffer);
  }
 
  public void blue()
  {
    for(var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
   
    m_led.setData(m_ledBuffer);
  }
 
  public void yellow()
  {
    for(var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, 255, 255, 0);
    }
   
    m_led.setData(m_ledBuffer);
  }
 
  public void updateHopperStrip()
  {
    int balls = Robot.hopper.getBalls();
    if(balls <= 5 && balls > 0)
    {
      for(int i = 0; i < m_ledBuffer.getLength() * balls / 5; i++)
      {
        m_ledBuffer.setRGB(i, 204, 39, 136);
        // (98,174,197)
      }

      for(int i = m_ledBuffer.getLength() * balls / 5; i < m_ledBuffer.getLength(); i++)
      {
        m_ledBuffer.setRGB(i, 14, 19, 151);
        //(230,64,114)
      }
    }
  }

  public void rainbow() 
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
    switch(currColor){

      case RAINBOW:
        rainbow();

        break;
      case MVRT:
        if(System.currentTimeMillis() - lastTime >= 50) 
        {
          rotate();
          lastTime = System.currentTimeMillis();
        }

        break;
      case PURPLE:
        purple();

        break;
      case YELLOW:
        yellow();

        break;
      case BLUE:
        if(Robot.hopper.getBalls() != 0 ){
          setColor(LEDColor.COUNTER);
        }
        blue();
        break;

      case COUNTER:
        if(Robot.hopper.getBalls() == 0){
          setColor(LEDColor.BLUE);
        }else{
          updateHopperStrip();
        }
        break;

      case RED:
        if(Robot.hopper.getBalls() >= 0 && Robot.hopper.getBalls() <= 5){
          setColor(LEDColor.COUNTER);
        }
        red();
    }

    m_led.setData(m_ledBuffer);

  }

}
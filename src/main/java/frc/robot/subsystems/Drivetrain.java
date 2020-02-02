/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  double lspeed;
  double rspeed; 
  Color lastColor = null;
  int NUM_COUNTS = 2;
  //boolean same = false;

  
  public Drivetrain() {
    Hardware.left = new TalonSRX(41);
    Hardware.right = new TalonSRX(22);
    Hardware.top = new TalonSRX(17);

    Hardware.top.setNeutralMode(NeutralMode.Brake);
    Hardware.left.setNeutralMode(NeutralMode.Brake);
    Hardware.right.setNeutralMode(NeutralMode.Brake);
  }
 
  public void topDrive(double speed){
    Hardware.top.set(ControlMode.PercentOutput, speed);

  }
  public void drive (){
    
      lspeed = Robot.oi.getY();
      rspeed = Robot.oi.getX();
    
    if(Robot.oi.getX()>0){
      lspeed = Robot.oi.getY();
      rspeed = (1-Robot.oi.getX())*Robot.oi.getY();
    }
    else if(Robot.oi.getX()<=0){
      rspeed = Robot.oi.getY();
      lspeed = (1+Robot.oi.getX())*Robot.oi.getY();
    }

    Hardware.left.set(ControlMode.PercentOutput, -lspeed+0.1);
    Hardware.right.set(ControlMode.PercentOutput, rspeed-0.1);
    //Hardware.left.set(ControlMode.PercentOutput, 0.1);
    //Hardware.right.set(ControlMode.PercentOutput, -0.1);
  }


 public void stop(){

  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

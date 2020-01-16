/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.Limelight;
import java.util.function.DoubleFunction;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase 
{
  	public TalonFX frontLeft, backLeft, frontRight, backRight;	

	private double quickStopAccumulator = 0.0;
	private double wheelDeadBand = 0.03;
	private double throttleDeadBand = 0.02;
	private static final double SENSITIVITY = 0.90;
	private DoubleFunction<Double> limiter = limiter(-0.9, 0.9);
	private SupplyCurrentLimitConfiguration driveMotorCurrentConfig; 

	public Drivetrain() 
	{
		frontLeft = new TalonFX(5);
		backLeft = new TalonFX(3);
		frontRight = new TalonFX(1);
		backRight= new TalonFX(2); 

		frontLeft.configFactoryDefault();
		frontRight.configFactoryDefault();
		backLeft.configFactoryDefault();
		backRight.configFactoryDefault();


		frontLeft.setInverted(false);
		backLeft.setInverted(false);
		frontRight.setInverted(true);
		backRight.setInverted(true);

		driveMotorCurrentConfig = new SupplyCurrentLimitConfiguration(true, 40, 80, .75);
		
		frontLeft.configSupplyCurrentLimit(driveMotorCurrentConfig);
		backLeft.configSupplyCurrentLimit(driveMotorCurrentConfig);
		frontRight.configSupplyCurrentLimit(driveMotorCurrentConfig);
		backRight.configSupplyCurrentLimit(driveMotorCurrentConfig);


		frontLeft.setSelectedSensorPosition(0);
		frontRight.setSelectedSensorPosition(0);

		frontLeft.setSensorPhase(false);
		frontRight.setSensorPhase(true);
	}

	public void cheesyDriveWithJoystick(double throttle, double wheel, boolean quickturn) 
	{
		wheel = handleDeadband(wheel, wheelDeadBand);
		throttle = handleDeadband(throttle, throttleDeadBand);

		double overPower;
		double angularPower;

		wheel = dampen(wheel, 0.5);
		wheel = dampen(wheel, 0.5);
		wheel = dampen(wheel, 0.5);

		if(quickturn)
		{
			if (Math.abs(throttle) < 0.2) 
			{
				double alpha = 0.1;
				quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limiter.apply(wheel) * 2;
			}
			overPower = 1.0;
			angularPower = wheel;
			angularPower *= 0.45;
		} 
		else 
		{
			overPower = 0.0;
			angularPower = Math.abs(throttle) * wheel * SENSITIVITY - quickStopAccumulator;
			angularPower *= 0.8;
			if(quickStopAccumulator > 1) 
			{
       			quickStopAccumulator -= 1;
			} 
			else if(quickStopAccumulator < -1) 
			{
        		quickStopAccumulator += 1;
			} 
			else
			{
       			quickStopAccumulator = 0.0;
      		}
    	}

		double rightPwm = throttle - (-1 * angularPower);
		double leftPwm = throttle + (-1 *angularPower);
		if(leftPwm > 1.0) 
		{
      		rightPwm -= overPower * (leftPwm - 1.0);
      		leftPwm = 1.0;
		} 
		else if(rightPwm > 1.0) 
		{
    		leftPwm -= overPower * (rightPwm - 1.0);
      		rightPwm = 1.0;
		} 
		else if (leftPwm < -1.0) 
		{
      		rightPwm += overPower * (-1.0 - leftPwm);
      		leftPwm = -1.0;
		} 
		else if (rightPwm < -1.0) 
		{
      		leftPwm += overPower * (-1.0 - rightPwm);
      		rightPwm = -1.0;
    	}

    	setLeftRightMotorOutputs( leftPwm, (rightPwm));
	}

	public double handleDeadband(double val, double deadband)
	{
    	return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}
	
	private static double dampen(double wheel, double wheelNonLinearity) 
	{
    	double factor = Math.PI * wheelNonLinearity;
    	return Math.sin(factor * wheel) / Math.sin(factor);
	}
	
	private DoubleFunction<Double> limiter(double minimum, double maximum) 
	{
		if (maximum < minimum) 
		{
      		throw new IllegalArgumentException("The minimum value cannot exceed the maximum value");
    	}
		return (double value) -> 
		{
			if (value > maximum) 
			{
        		return maximum;
      		}
			if (value < minimum) 
			{
        		return minimum;
      		}
      	return value;
    	};
	}
	
	public void cheesyIshDrive(double throttle, double wheel, boolean quickTurn)
	{
		double left = 0, right = 0;

		SmartDashboard.putNumber("Whee;", wheel);
		final double kWheelGain = 0.05;
        final double kWheelNonlinearity = 0.05;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

		wheel *= kWheelGain;
		Twist2d motion = new Twist2d(throttle, 0, wheel);
		if (Math.abs(motion.dtheta) < 1E-9) 
		{
      		left = motion.dx;
      		right = motion.dx;
		}
		else
		{
			double delta_v = 29 * motion.dtheta / (2* Constants.kTrackScrubFactor);
		//	double delta_v = 30.70866 * motion.dtheta / (2 * 1.0469745223);
			left = motion.dx + delta_v;
			right = motion.dx - delta_v;

		}	
		
		SmartDashboard.putNumber("Left", left);
		SmartDashboard.putNumber("right", right);
		double scaling_factor = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
		setLeftRightMotorOutputs(left/scaling_factor, right/scaling_factor);
	}

	public void setLeftRightMotorOutputs(double left, double right)
	{
		frontLeft.set(ControlMode.PercentOutput, left);
		backLeft.set(ControlMode.PercentOutput, left);
		frontRight.set(ControlMode.PercentOutput, right);
		backRight.set(ControlMode.PercentOutput, right);
	}

	public void driveWithTarget(double angle)
	{
		cheesyIshDrive(0, angle / 30, true);
		/*
		double left = 0, right = 0;
		if(angle != 0)
		{
			left = -angle / 30;
			right = angle / 30;
		}
		setLeftRightMotorOutputs(left, right); */
	}
	
	public void stop() 
	{
		setLeftRightMotorOutputs(0, 0);
	}

}
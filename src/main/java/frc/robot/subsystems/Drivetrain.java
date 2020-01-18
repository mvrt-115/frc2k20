/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Hardware;
import java.util.function.DoubleFunction;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase 
{

	private double quickStopAccumulator = 0.0;
	private static final double SENSITIVITY = 0.90;
	private DoubleFunction<Double> limiter = limiter(-0.9, 0.9);
	private SupplyCurrentLimitConfiguration driveMotorCurrentConfig; 

	public Drivetrain() 
	{
		Hardware.frontLeft = new TalonFX(5);
		Hardware.backLeft = new TalonFX(3);
		Hardware.frontRight = new TalonFX(1);
		Hardware.backRight= new TalonFX(2); 

		Hardware.gyro = new AHRS(SPI.Port.kMXP);

		Hardware.frontLeft.configFactoryDefault();
		Hardware.frontRight.configFactoryDefault();
		Hardware.backLeft.configFactoryDefault();
		Hardware.backRight.configFactoryDefault();


		Hardware.frontLeft.setInverted(false);
		Hardware.backLeft.setInverted(false);
		Hardware.frontRight.setInverted(true);
		Hardware.backRight.setInverted(true);

		driveMotorCurrentConfig = new SupplyCurrentLimitConfiguration(true, 40, 80, .75);
		
		Hardware.frontLeft.configSupplyCurrentLimit(driveMotorCurrentConfig);
		Hardware.backLeft.configSupplyCurrentLimit(driveMotorCurrentConfig);
		Hardware.frontRight.configSupplyCurrentLimit(driveMotorCurrentConfig);
		Hardware.backRight.configSupplyCurrentLimit(driveMotorCurrentConfig);

		resetEncoder();
		configNeutralMode(NeutralMode.Coast);

		Hardware.frontLeft.setSensorPhase(false);
		Hardware.backLeft.setSensorPhase(false);
		Hardware.frontRight.setSensorPhase(true);
		Hardware.backRight.setSensorPhase(true);

		

		Hardware.gyro.reset();
	}

	public void resetEncoder(){
		Hardware.frontLeft.setSelectedSensorPosition(0);
		Hardware.frontRight.setSelectedSensorPosition(0);
		Hardware.backLeft.setSelectedSensorPosition(0);
		Hardware.backRight.setSelectedSensorPosition(0);
	}

	public void configNeutralMode(NeutralMode mode){
		Hardware.frontLeft.setNeutralMode(mode);
		Hardware.frontRight.setNeutralMode(mode);
		Hardware.backLeft.setNeutralMode(mode);
		Hardware.backRight.setNeutralMode(mode);
	}

	public void cheesyDriveWithJoystick(double throttle, double wheel, boolean quickturn) 
	{
		wheel = handleDeadband(wheel, Constants.kWheelDeadband);
		throttle = handleDeadband(throttle, Constants.kThrottleDeadband);

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

    	setLeftRightMotorOutputs( leftPwm, rightPwm);
	}

/*	public double handleDeadband(double val, double deadband)
	{
    	return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}
*/	

	public double handleDeadband(double val, double deadband){
		if(Math.abs(val) >= deadband) 
			return (val - deadband * Math.abs(val) / val) / (1 - deadband);
		else
			return 0;
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
		
	//	throttle = handleDeadband(throttle, Constants.kThrottleDeadband);
	//	wheel = handleDeadband(wheel, Constants.kWheelDeadband);

		double left = 0, right = 0;

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
			double delta_v = Constants.kTrackWidthInches * motion.dtheta / (2* Constants.kTrackScrubFactor);
			left = motion.dx + delta_v;
			right = motion.dx - delta_v;

		}	
		

		double scaling_factor = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
		setLeftRightMotorOutputs(left/scaling_factor, right/scaling_factor);
	}

	public void setLeftRightMotorOutputs(double left, double right)
	{
		Hardware.frontLeft.set(ControlMode.PercentOutput, left);
		Hardware.backLeft.set(ControlMode.PercentOutput, left);
		Hardware.frontRight.set(ControlMode.PercentOutput, right);
		Hardware.backRight.set(ControlMode.PercentOutput, right);
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

	public void log(){
		SmartDashboard.putNumber("Left Encoder", Hardware.frontLeft.getSelectedSensorPosition());
		SmartDashboard.putNumber("Right Encoder", Hardware.frontRight.getSelectedSensorPosition());
		SmartDashboard.putNumber("Current", Hardware.frontLeft.getStatorCurrent());

	}
}
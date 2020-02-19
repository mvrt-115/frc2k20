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
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase {

	private double quickStopAccumulator = 0.0;
	private DoubleFunction<Double> limiter = limiter(-0.9, 0.9);
	private SupplyCurrentLimitConfiguration driveMotorCurrentConfig;

	private DifferentialDriveKinematics driveKinematics;
	private DifferentialDriveOdometry driveOdometry;
	private SimpleMotorFeedforward driveFeedforward;
	private PIDController leftDriveController, rightDriveController;
	private RamseteController ramseteController;
	private TrajectoryConfig trajectoryConfig;
	private Pose2d currPosition;

	public Drivetrain() {
		Hardware.leftMaster = new TalonFX(2);
		Hardware.leftFollower = new TalonFX(1);
		Hardware.rightMaster = new TalonFX(3);
		Hardware.rightFollower = new TalonFX(5);

		Hardware.gyro = new AHRS(SPI.Port.kMXP);

		Hardware.leftMaster.configFactoryDefault();
		Hardware.rightMaster.configFactoryDefault();
		Hardware.leftFollower.configFactoryDefault();
		Hardware.rightFollower.configFactoryDefault();

		Hardware.leftMaster.setInverted(false);
		Hardware.leftFollower.setInverted(false);
		Hardware.rightMaster.setInverted(true);
		Hardware.rightFollower.setInverted(true);


		Hardware.leftMaster.configVoltageCompSaturation(10, Constants.kTimeoutMs);
		Hardware.leftFollower.configVoltageCompSaturation(10, Constants.kTimeoutMs);
		Hardware.rightMaster.configVoltageCompSaturation(10, Constants.kTimeoutMs);
		Hardware.rightFollower.configVoltageCompSaturation(10, Constants.kTimeoutMs);

		Hardware.leftMaster.enableVoltageCompensation(true);
		Hardware.leftFollower.enableVoltageCompensation(true);
		Hardware.rightMaster.enableVoltageCompensation(true);
		Hardware.rightFollower.enableVoltageCompensation(true);

		driveMotorCurrentConfig = new SupplyCurrentLimitConfiguration(true, 40, 50, 3.8);

		Hardware.leftMaster.configSupplyCurrentLimit(driveMotorCurrentConfig);
		Hardware.leftFollower.configSupplyCurrentLimit(driveMotorCurrentConfig);
		Hardware.rightMaster.configSupplyCurrentLimit(driveMotorCurrentConfig);
		Hardware.rightFollower.configSupplyCurrentLimit(driveMotorCurrentConfig);

		Hardware.leftMaster.configOpenloopRamp(.4);
		Hardware.leftFollower.configOpenloopRamp(.4);
		Hardware.rightMaster.configOpenloopRamp(.4);
		Hardware.rightFollower.configOpenloopRamp(.4);

		Hardware.leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
				Constants.kTimeoutMs);
		Hardware.rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
				Constants.kTimeoutMs);
		Hardware.leftFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
				Constants.kTimeoutMs);
		Hardware.rightFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
				Constants.kTimeoutMs);

		resetEncoder();
		configNeutralMode(NeutralMode.Coast, NeutralMode.Coast);
		Hardware.gyro.reset();

		driveKinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);
		driveOdometry = new DifferentialDriveOdometry(getGyroRotation());
		driveFeedforward = new SimpleMotorFeedforward(Constants.kDriveS, Constants.kDriveV, Constants.kDriveA);
		leftDriveController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
		rightDriveController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
		currPosition = new Pose2d();

		trajectoryConfig = new TrajectoryConfig(Constants.kMaxVelocityMetersPerSecond,
				Constants.kMaxAccelerationMetersPerSecondSq);
		trajectoryConfig.setReversed(false);

		ramseteController = new RamseteController();

	}

	@Override
	public void periodic() {
		driveOdometry.update(getGyroRotation(), getLeftEncoderDistance(), getRightEncoderDistance());
		currPosition = driveOdometry.getPoseMeters();
	}

	public void resetOdometry() {
		setOdometry(new Pose2d());
	}

	public void setOdometry(Pose2d newPose) {
		currPosition = newPose;
		resetEncoder();
		Hardware.gyro.reset();
		driveOdometry.resetPosition(newPose, getGyroRotation());
	}

	public double getLeftEncoderDistance() {

		double motorTicks = (Hardware.leftMaster.getSelectedSensorPosition()
				+ Hardware.leftFollower.getSelectedSensorPosition()) / 2;
		return motorTicks / Constants.kFalconTicksPerRotation / Constants.kDriveGearRatio  * Math.PI
				* Constants.kWheelDiameterMeters;
	}

	public double getRightEncoderDistance() {
		double motorTicks = (Hardware.rightMaster.getSelectedSensorPosition()
				+ Hardware.rightFollower.getSelectedSensorPosition()) / 2;

		return motorTicks / Constants.kFalconTicksPerRotation / Constants.kDriveGearRatio  * Math.PI
				* Constants.kWheelDiameterMeters;
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(
				Hardware.leftMaster.getSelectedSensorVelocity() / Constants.kFalconTicksPerRotation
						/ Constants.kDriveGearRatio * Math.PI * Constants.kWheelDiameterMeters * 10,
				Hardware.leftMaster.getSelectedSensorVelocity() / Constants.kFalconTicksPerRotation
						/ Constants.kDriveGearRatio * Math.PI * Constants.kWheelDiameterMeters * 10);
	}

	public void setOutputVolts(double leftVolts, double rightVolts) {
		setLeftRightMotorOutputs(leftVolts / 10.0, rightVolts / 10.0);
	}

	public Rotation2d getGyroRotation() {
		return Rotation2d.fromDegrees(-Hardware.gyro.getAngle());
	}

	public void resetEncoder() {
		Hardware.leftMaster.setSelectedSensorPosition(0);
		Hardware.rightMaster.setSelectedSensorPosition(0);
		Hardware.leftFollower.setSelectedSensorPosition(0);
		Hardware.rightFollower.setSelectedSensorPosition(0);
	}

	public void configNeutralMode(NeutralMode _mode){
		configNeutralMode(_mode, _mode);
	}
	public void configNeutralMode(NeutralMode mode1, NeutralMode mode2) {
		Hardware.leftMaster.setNeutralMode(mode1);
		Hardware.rightMaster.setNeutralMode(mode1);
		Hardware.leftFollower.setNeutralMode(mode2);
		Hardware.rightFollower.setNeutralMode(mode2);
	}

	public void cheesyDriveWithJoystick(double throttle, double wheel, boolean quickturn) {
		wheel = handleDeadband(wheel, Constants.kWheelDeadband);
		throttle = handleDeadband(throttle, Constants.kThrottleDeadband);

		double overPower;
		double angularPower;

		wheel = dampen(wheel, 0.5);
		wheel = dampen(wheel, 0.5);
		wheel = dampen(wheel, 0.5);

		if (quickturn) {
			if (Math.abs(throttle) < 0.2) {
				double alpha = 0.1;
				quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limiter.apply(wheel) * 2;
			}
			overPower = 1.0;
			angularPower = wheel;
			angularPower *= 0.45;
		} else {
			overPower = 0.0;
			angularPower = Math.abs(throttle) * wheel * Constants.kSensitivity - quickStopAccumulator;
			angularPower *= 0.8;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0.0;
			}
		}

		double rightPwm = throttle - (-1 * angularPower);
		double leftPwm = throttle + (-1 * angularPower);
		if (leftPwm > 1.0) {
			rightPwm -= overPower * (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
			leftPwm -= overPower * (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
			rightPwm += overPower * (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
			leftPwm += overPower * (-1.0 - rightPwm);
			rightPwm = -1.0;
		}

		setLeftRightMotorOutputs(leftPwm, rightPwm);
	}

	/*
	 * public double handleDeadband(double val, double deadband) { return
	 * (Math.abs(val) > Math.abs(deadband)) ? val : 0.0; }
	 */

	public double handleDeadband(double val, double deadband) {
		if (Math.abs(val) >= deadband)
			return (val - deadband * Math.abs(val) / val) / (1 - deadband);
		else
			return 0;
	}

	private static double dampen(double wheel, double wheelNonLinearity) {
		double factor = Math.PI * wheelNonLinearity;
		return Math.sin(factor * wheel) / Math.sin(factor);
	}

	private DoubleFunction<Double> limiter(double minimum, double maximum) {
		if (maximum < minimum) {
			throw new IllegalArgumentException("The minimum value cannot exceed the maximum value");
		}
		return (double value) -> {
			if (value > maximum) {
				return maximum;
			}
			if (value < minimum) {
				return minimum;
			}
			return value;
		};
	}

	public void cheesyIshDrive(double throttle, double wheel, boolean quickTurn) {

		throttle = handleDeadband(throttle, Constants.kThrottleDeadband);
		wheel = handleDeadband(wheel, Constants.kWheelDeadband);

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
		if (Math.abs(motion.dtheta) < 1E-9) {
			left = motion.dx;
			right = motion.dx;
		} else {
			double delta_v = Constants.kTrackWidthInches * motion.dtheta / (2 * Constants.kTrackScrubFactor);
			left = motion.dx + delta_v;
			right = motion.dx - delta_v;

		}

		double scaling_factor = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
		setLeftRightMotorOutputs(left / scaling_factor, right / scaling_factor);
	}

	public void setLeftRightMotorOutputs(double left, double right) {
		Hardware.leftMaster.set(ControlMode.PercentOutput, left);
		Hardware.leftFollower.set(ControlMode.PercentOutput, left);
		Hardware.rightFollower.set(ControlMode.PercentOutput, right);
		Hardware.rightMaster.set(ControlMode.PercentOutput, right);

	}

	public void alignToTarget(double error) {
		double kFF = 0.037;
		double kP = .003;
		double output;

		if (Math.abs(error) > .5) {
			output = error * kP + Math.copySign(kFF, error);
		} else {
			output = error * kP;
		}

		SmartDashboard.putNumber("Speed", output);
		SmartDashboard.putNumber("Offset", error);
		setLeftRightMotorOutputs(output, -output);
	}

	public void stop() {
		setLeftRightMotorOutputs(0, 0);
	}

	public void log() {//
		// SmartDashboard.putNumber("Left Encoder", Hardware.leftMaster.getSelectedSensorPosition());
		// SmartDashboard.putNumber("Right Encoder", Hardware.rightMaster.getSelectedSensorPosition());
		SmartDashboard.putNumber("Current", Hardware.leftMaster.getStatorCurrent()* 4);
		 SmartDashboard.putNumber("Curr X Position", currPosition.getTranslation().getX());
		 SmartDashboard.putNumber("Curr Y Position", currPosition.getTranslation().getY());
		 SmartDashboard.putNumber("NavX", Hardware.gyro.getAngle());
	}

	public void invertPathDirection(boolean reversed){
		trajectoryConfig.setReversed(reversed);
	}

	public PIDController getLeftDriveController() {
		return leftDriveController;
	}

	public PIDController getRightDriveController() {
		return rightDriveController;
	}

	public RamseteController getRamseteController() {
		return ramseteController;
	}

	public TrajectoryConfig getTrajectoryConfig() {
		return trajectoryConfig;
	}

	public Pose2d getPose() {
		return currPosition;
	}

	public SimpleMotorFeedforward getFeedForward() {
		return driveFeedforward;
	}

	public DifferentialDriveKinematics getDriveKinematics() {
		return driveKinematics;
	}
}
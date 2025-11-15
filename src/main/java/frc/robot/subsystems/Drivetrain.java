// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;

public class Drivetrain extends SubsystemBase {

  private WPI_TalonSRX m_frontLeftMotor, m_frontRightMotor, m_backLeftMotor, m_backRightMotor;
  private AHRS m_NavX;
  private Encoder m_frontLeftEncoder, m_frontRightEncoder, m_backLeftEncoder, m_backRightEncoder;
  private MecanumDriveKinematics m_mecanumDriveKinematics;
  private SlewRateLimiter m_frontLeftLimiter, m_frontRightLimiter, m_backLeftLimiter, m_backRightLimiter;
  private PIDController m_frontLeftPidController, m_frontRightPidController, m_backLeftPidController,
      m_backRightPidController;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_frontLeftMotor = new WPI_TalonSRX(10);
    m_backLeftMotor = new WPI_TalonSRX(11);
    m_frontRightMotor = new WPI_TalonSRX(12);
    m_backRightMotor = new WPI_TalonSRX(13);

    m_frontLeftMotor.configAllSettings(new TalonSRXConfiguration());
    m_frontLeftMotor.setInverted(DrivetrainConstants.kFrontLeftInverted);
    m_frontRightMotor.configAllSettings(new TalonSRXConfiguration());
    m_frontRightMotor.setInverted(DrivetrainConstants.kFrontRightInverted);
    m_backLeftMotor.configAllSettings(new TalonSRXConfiguration());
    m_backLeftMotor.setInverted(DrivetrainConstants.kBackLeftInverted);
    m_backRightMotor.configAllSettings(new TalonSRXConfiguration());
    m_backRightMotor.setInverted(DrivetrainConstants.kBackRightInverted);

    m_frontLeftLimiter = new SlewRateLimiter(DrivetrainConstants.kSlewRateLimitForMotors);
    m_frontRightLimiter = new SlewRateLimiter(DrivetrainConstants.kSlewRateLimitForMotors);
    m_backLeftLimiter = new SlewRateLimiter(DrivetrainConstants.kSlewRateLimitForMotors);
    m_backRightLimiter = new SlewRateLimiter(DrivetrainConstants.kSlewRateLimitForMotors);

    m_frontLeftPidController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
        DrivetrainConstants.kD);
    m_frontRightPidController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
        DrivetrainConstants.kD);
    m_backLeftPidController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
    m_backRightPidController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI,
        DrivetrainConstants.kD);

    m_NavX = new AHRS(NavXComType.kMXP_SPI);

    m_NavX.reset();

    m_backLeftEncoder = new Encoder(0, 1);
    m_backRightEncoder = new Encoder(2, 3);
    m_frontLeftEncoder = new Encoder(4, 5);
    m_frontRightEncoder = new Encoder(6, 7);

    m_backLeftEncoder.setDistancePerPulse(7.0 / 20.0);
    m_backRightEncoder.setDistancePerPulse(7.0 / 20.0);
    m_frontLeftEncoder.setDistancePerPulse(7.0 / 20.0);
    m_frontRightEncoder.setDistancePerPulse(7.0 / 20.0);

    m_mecanumDriveKinematics = new MecanumDriveKinematics(
        new Translation2d(DrivetrainConstants.kWheelBase.div(2), DrivetrainConstants.kTrackwidth.div(-2)),
        new Translation2d(DrivetrainConstants.kWheelBase.div(2), DrivetrainConstants.kTrackwidth.div(2)),
        new Translation2d(DrivetrainConstants.kWheelBase.div(-2), DrivetrainConstants.kTrackwidth.div(-2)),
        new Translation2d(DrivetrainConstants.kWheelBase.div(-2), DrivetrainConstants.kTrackwidth.div(2)));
  }

  /**
   * @return Yaw from NavX in Rotation2d
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_NavX.getYaw()).plus(Rotation2d.fromDegrees(90));
  }

  /**
   * Reset the gyro
   */
  public void resetGyro() {
    m_NavX.zeroYaw();
  }

  /**
   * Drive the robot
   * 
   * @param speedX forward speed in meters per second
   * @param speedY rightwards speed in meters per second
   * @param speedR clockwise speed in radians per second
   */
  public void drive(double speedX, double speedY, double speedR) {
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, speedR);
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
    applyWheelSpeeds(m_mecanumDriveKinematics.toWheelSpeeds(speeds));

  }

  /**
   * For driving using the controller
   * 
   * @param forwardInput  Speed forward/backward to go (Left joystick
   *                      forward/backward)
   * @param sidewaysInput Speed left/right to go (Left joystick left/right)
   * @param turnInput     Speed to rotate (Right joystick left/right)
   * @param boost         Go full speed or slow speed?
   */
  public void userDrive(double forwardInput, double sidewaysInput, double turnInput, boolean boost) {
    double xSpeed = boost ? forwardInput * DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond)
        : forwardInput * OperatorConstants.kNormalWheelSpeed.in(MetersPerSecond);
    double ySpeed = boost ? sidewaysInput * DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond)
        : sidewaysInput * OperatorConstants.kNormalWheelSpeed.in(MetersPerSecond);
    double rSpeed = boost ? turnInput * DrivetrainConstants.kMaxRobotTurnSpeed.in(RadiansPerSecond)
        : turnInput * OperatorConstants.kRobotTurnSpeed.in(RadiansPerSecond);

    SmartDashboard.putNumber("input xSpeed:", xSpeed);
    SmartDashboard.putNumber("input ySpeed", ySpeed);
    SmartDashboard.putNumber("input rSpeed", rSpeed);
    drive(xSpeed, ySpeed, rSpeed);

  }

  /**
   * Set the wheels to go the speed you want
   * 
   * @param wheelSpeeds Wheel speeds to go
   */
  public void applyWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {

    double frontLeftIntendedSpeed = m_frontLeftLimiter.calculate(wheelSpeeds.frontLeftMetersPerSecond);
    double frontRightIntendedSpeed = m_frontRightLimiter.calculate(wheelSpeeds.frontRightMetersPerSecond);
    double backLeftIntendedSpeed = m_backLeftLimiter.calculate(wheelSpeeds.rearLeftMetersPerSecond);
    double backRightIntendedSpeed = m_backRightLimiter.calculate(wheelSpeeds.rearRightMetersPerSecond);

    SmartDashboard.putNumber("FL intended speed", frontLeftIntendedSpeed);
    SmartDashboard.putNumber("FR intended speed", frontRightIntendedSpeed);
    SmartDashboard.putNumber("BL intended speed", backLeftIntendedSpeed);
    SmartDashboard.putNumber("BR intended speed", backRightIntendedSpeed);

    SmartDashboard.putNumber("MaxwheelSpeed in meters per sec", DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond));
    wheelSpeeds.desaturate(DrivetrainConstants.kMaxWheelSpeed);
    m_frontLeftMotor
        .setVoltage(frontLeftIntendedSpeed / DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) * 12);
    m_frontRightMotor.setVoltage(
        frontRightIntendedSpeed / DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) * 12);
    m_backLeftMotor
        .setVoltage(backLeftIntendedSpeed / DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) * 12);
    m_backRightMotor
        .setVoltage(backRightIntendedSpeed / DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) * 12);

    SmartDashboard.putNumber("MaxWheelSPeed", DrivetrainConstants.kWheelMaxAngularVelocity.in(RotationsPerSecond));
    SmartDashboard.putNumber("Circ", DrivetrainConstants.kWheelCircumference.in(Meter));
    SmartDashboard.putNumber("FrontLeftVoltage", m_frontLeftMotor.get() * m_frontLeftMotor.getBusVoltage());
    SmartDashboard.putNumber("FrontRightVoltage", m_frontRightMotor.get() * m_frontRightMotor.getBusVoltage());
    SmartDashboard.putNumber("BackLeftVoltage", m_backLeftMotor.get() * m_backLeftMotor.getBusVoltage());
    SmartDashboard.putNumber("BackRightVoltage", m_backRightMotor.get() * m_backRightMotor.getBusVoltage());
    SmartDashboard.putNumber("FrontLeftAmps", m_frontLeftMotor.getStatorCurrent());
    SmartDashboard.putNumber("FrontRightAmps", m_frontRightMotor.getStatorCurrent());
    SmartDashboard.putNumber("BackLeftAmps", m_backLeftMotor.getStatorCurrent());
    SmartDashboard.putNumber("BackRightAmps", m_backRightMotor.getStatorCurrent());
    SmartDashboard.putNumber("Front Left Motor Speed:", m_frontLeftEncoder.getRate());
    SmartDashboard.putNumber("Front Right Motor Speed:", m_frontRightEncoder.getRate());
    SmartDashboard.putNumber("Back Left Motor Speed:", m_backLeftEncoder.getRate());
    SmartDashboard.putNumber("Back Right Motor Speed:", m_backLeftEncoder.getRate());
  }

  public void applyWheelSpeedsPID(MecanumDriveWheelSpeeds wheelSpeeds) {

    double frontLeftIntendedSpeed = m_frontLeftLimiter.calculate(wheelSpeeds.frontLeftMetersPerSecond);
    double frontRightIntendedSpeed = m_frontRightLimiter.calculate(wheelSpeeds.frontRightMetersPerSecond);
    double backLeftIntendedSpeed = m_backLeftLimiter.calculate(wheelSpeeds.rearLeftMetersPerSecond);
    double backRightIntendedSpeed = m_backRightLimiter.calculate(wheelSpeeds.rearRightMetersPerSecond);

    wheelSpeeds.desaturate(DrivetrainConstants.kMaxWheelSpeed);
    m_frontLeftMotor
        .setVoltage(m_frontLeftPidController.calculate(frontLeftIntendedSpeed));
    m_frontRightMotor.setVoltage(
        m_frontRightPidController.calculate(frontRightIntendedSpeed));
    m_backLeftMotor
        .setVoltage(m_backLeftPidController.calculate(backLeftIntendedSpeed));
    m_backRightMotor
        .setVoltage(m_backRightPidController.calculate(backRightIntendedSpeed));

    SmartDashboard.putNumber("Front Left Motor Speed:", m_frontLeftEncoder.getRate());
    SmartDashboard.putNumber("Front Right Motor Speed:", m_frontRightEncoder.getRate());
    SmartDashboard.putNumber("Back Left Motor Speed:", m_backLeftEncoder.getRate());
    SmartDashboard.putNumber("Back Right Motor Speed:", m_backLeftEncoder.getRate());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

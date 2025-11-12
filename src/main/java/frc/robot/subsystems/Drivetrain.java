// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;

public class Drivetrain extends SubsystemBase {

  private WPI_TalonSRX m_frontLeftMotor, m_frontRightMotor, m_backLeftMotor, m_backRightMotor;
  private AHRS m_NavX;
  private Encoder m_frontLeftEncoder, m_frontRightEncoder, m_backLeftEncoder, m_backRightEncoder;
  private MecanumDriveKinematics m_mecanumDriveKinematics;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_frontLeftMotor = new WPI_TalonSRX(10);
    m_backLeftMotor = new WPI_TalonSRX(11);
    m_frontRightMotor = new WPI_TalonSRX(12);
    m_backRightMotor = new WPI_TalonSRX(13);

    m_NavX = new AHRS(NavXComType.kMXP_SPI);

    m_NavX.reset();

    m_backLeftEncoder = new Encoder(0, 1);
    m_backRightEncoder = new Encoder(2, 3);
    m_frontLeftEncoder = new Encoder(4, 5);
    m_frontRightEncoder = new Encoder(6, 7);

    m_backLeftEncoder.setDistancePerPulse(7.0/20.0);
    m_backRightEncoder.setDistancePerPulse(7.0/20.0);
    m_frontLeftEncoder.setDistancePerPulse(7.0/20.0);
    m_frontRightEncoder.setDistancePerPulse(7.0/20.0);

    m_mecanumDriveKinematics = new MecanumDriveKinematics( 
      new Translation2d(DrivetrainConstants.kWheelBase.div(2), DrivetrainConstants.kTrackwidth.div(-2)),
      new Translation2d(DrivetrainConstants.kWheelBase.div(2), DrivetrainConstants.kTrackwidth.div(2)),
      new Translation2d(DrivetrainConstants.kWheelBase.div(-2), DrivetrainConstants.kTrackwidth.div(-2)),
      new Translation2d(DrivetrainConstants.kWheelBase.div(-2), DrivetrainConstants.kTrackwidth.div(2)));
  }

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(m_NavX.getYaw());
  }
  /**
   * Drive the robot
   * @param speedX forward speed in meters per second
   * @param speedY rightwards speed in meters per second
   * @param speedR clockwise speed in radians per second
   */
  public void drive(double speedX, double speedY, double speedR) {
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, speedR);
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
    applyWheelSpeeds(m_mecanumDriveKinematics.toWheelSpeeds(speeds));

  }

  public void userDrive(double forwardInput, double sidewaysInput, double turnInput, boolean boost){
    double xSpeed = boost ? forwardInput * DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) : forwardInput * OperatorConstants.kNormalWheelSpeed.in(MetersPerSecond);
    double ySpeed = boost ? sidewaysInput * DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) : sidewaysInput * OperatorConstants.kNormalWheelSpeed.in(MetersPerSecond);
    double rSpeed = boost ? turnInput * DrivetrainConstants.kMaxRobotTurnSpeed.in(RadiansPerSecond) : turnInput * OperatorConstants.kRobotTurnSpeed.in(RadiansPerSecond);
    drive(xSpeed, ySpeed, rSpeed);
  }

  public void applyWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds){
    wheelSpeeds.desaturate(DrivetrainConstants.kMaxWheelSpeed);
    m_frontLeftMotor.setVoltage(wheelSpeeds.frontLeftMetersPerSecond / DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) * 12);
    m_frontRightMotor.setVoltage(wheelSpeeds.frontRightMetersPerSecond / DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) * 12);
    m_backLeftMotor.setVoltage(wheelSpeeds.rearLeftMetersPerSecond / DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) * 12);
    m_backRightMotor.setVoltage(wheelSpeeds.rearRightMetersPerSecond / DrivetrainConstants.kMaxWheelSpeed.in(MetersPerSecond) * 12);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

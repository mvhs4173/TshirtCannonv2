// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Wheel;
import static frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.Constants.OperatorConstants;

public class Drivetrain extends SubsystemBase {

        private Wheel m_frontLeftWheel, m_frontRightWheel, m_backLeftWheel, m_backRightWheel;
        private AHRS m_NavX;
        private MecanumDriveKinematics m_mecanumDriveKinematics;
        private Config m_sysIdConfig;
        private Mechanism m_sysIdMechanism;
        private SysIdRoutine m_sysIdRoutine;

        private final MutVoltage m_appliedVoltage = Volts.mutable(0);
        private final MutDistance m_distance = Meters.mutable(0);
        private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

        /** Creates a new Drivetrain. */
        public Drivetrain() {
                m_sysIdConfig = new Config(kSysIdRampRate, kStepVoltage,
                                kTimeout);

                m_sysIdMechanism = new Mechanism(this::applyVolts, log -> {
                        log.motor("drive-front-left")
                                        .voltage(
                                                        m_appliedVoltage.mut_replace(
                                                                        m_frontLeftWheel.getVoltage(), Volts))
                                        .linearPosition(m_distance.mut_replace(m_frontLeftWheel.getDistanceMeters(),
                                                        Meters))
                                        .linearVelocity(
                                                        m_velocity.mut_replace(
                                                                        m_frontLeftWheel.getSpeedMetersPerSecond(),
                                                                        MetersPerSecond));
                        log.motor("drive-front-right")
                                        .voltage(
                                                        m_appliedVoltage.mut_replace(
                                                                        m_frontRightWheel.getVoltage(), Volts))
                                        .linearPosition(m_distance.mut_replace(m_frontRightWheel.getDistanceMeters(),
                                                        Meters))
                                        .linearVelocity(
                                                        m_velocity.mut_replace(
                                                                        m_frontRightWheel.getSpeedMetersPerSecond(),
                                                                        MetersPerSecond));
                        log.motor("drive-back-left")
                                        .voltage(
                                                        m_appliedVoltage.mut_replace(
                                                                        m_backLeftWheel.getVoltage(), Volts))
                                        .linearPosition(m_distance.mut_replace(m_backLeftWheel.getDistanceMeters(),
                                                        Meters))
                                        .linearVelocity(
                                                        m_velocity.mut_replace(
                                                                        m_backLeftWheel.getSpeedMetersPerSecond(),
                                                                        MetersPerSecond));
                        log.motor("drive-back-right")
                                        .voltage(
                                                        m_appliedVoltage.mut_replace(
                                                                        m_backRightWheel.getVoltage(), Volts))
                                        .linearPosition(m_distance.mut_replace(m_backRightWheel.getDistanceMeters(),
                                                        Meters))
                                        .linearVelocity(
                                                        m_velocity.mut_replace(
                                                                        m_backRightWheel.getSpeedMetersPerSecond(),
                                                                        MetersPerSecond));
                }, this, "sysIdRoutine");

                m_sysIdRoutine = new SysIdRoutine(m_sysIdConfig, m_sysIdMechanism);

                m_frontLeftWheel = new Wheel("Front Left Wheel",
                                kFrontLeftMotorID,
                                kFrontLeftMotorA,
                                kFrontLeftMotorB,
                                kFrontLeftInverted,
                                kFrontLeftEncoderInverted);
                ;
                m_frontRightWheel = new Wheel("Front Right Wheel",
                                kFrontRightMotorID,
                                kFrontRightMotorA,
                                kFrontRightMotorB,
                                kFrontRightInverted,
                                kFrontRightEncoderInverted);
                ;
                m_backLeftWheel = new Wheel("Back Left Wheel",
                                kBackLeftMotorID,
                                kBackLeftMotorA,
                                kBackLeftMotorB,
                                kBackLeftInverted,
                                kBackLeftEncoderInverted);
                ;
                m_backRightWheel = new Wheel("Back Right Wheel",
                                kBackRightMotorID,
                                kBackRightMotorA,
                                kBackRightMotorB,
                                kBackRightInverted,
                                kBackRightEncoderInverted);

                m_NavX = new AHRS(NavXComType.kMXP_SPI);

                m_NavX.reset();

                m_mecanumDriveKinematics = new MecanumDriveKinematics(
                                new Translation2d(kWheelBase.div(2), kTrackwidth.div(-2)),
                                new Translation2d(kWheelBase.div(2), kTrackwidth.div(2)),
                                new Translation2d(kWheelBase.div(-2), kTrackwidth.div(-2)),
                                new Translation2d(kWheelBase.div(-2), kTrackwidth.div(2)));
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
                forwardInput *= Math.abs(forwardInput) < OperatorConstants.kDeadzone ? 0 : Math.abs(forwardInput);
                sidewaysInput *= Math.abs(sidewaysInput) < OperatorConstants.kDeadzone ? 0 : Math.abs(sidewaysInput);
                turnInput *= Math.abs(turnInput) < OperatorConstants.kDeadzone ? 0 : Math.abs(turnInput);
                double xSpeed = boost ? forwardInput * kMaxWheelSpeed.in(MetersPerSecond)
                                : forwardInput * OperatorConstants.kNormalWheelSpeed.in(MetersPerSecond);
                double ySpeed = boost ? sidewaysInput * kMaxWheelSpeed.in(MetersPerSecond)
                                : sidewaysInput * OperatorConstants.kNormalWheelSpeed.in(MetersPerSecond);
                double rSpeed = boost ? turnInput * kMaxRobotTurnSpeed.in(RadiansPerSecond)
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
                wheelSpeeds.desaturate(kMaxWheelSpeed);
                m_frontLeftWheel
                                .applyVoltage(12 * wheelSpeeds.frontLeftMetersPerSecond
                                                / kMaxWheelSpeed.in(MetersPerSecond));
                m_frontRightWheel
                                .applyVoltage(12 * wheelSpeeds.frontRightMetersPerSecond
                                                / kMaxWheelSpeed.in(MetersPerSecond));
                m_backLeftWheel
                                .applyVoltage(12 * wheelSpeeds.rearLeftMetersPerSecond
                                                / kMaxWheelSpeed.in(MetersPerSecond));
                m_backRightWheel
                                .applyVoltage(12 * wheelSpeeds.rearRightMetersPerSecond
                                                / kMaxWheelSpeed.in(MetersPerSecond));

                SmartDashboard.putNumber("MaxWheelSpeed", kWheelMaxAngularVelocity.in(RotationsPerSecond));
        }

        public void applyWheelSpeedsFeedforward(MecanumDriveWheelSpeeds wheelSpeeds) {

                wheelSpeeds.desaturate(kMaxWheelSpeed);
                m_frontLeftWheel.applyWheelSpeed(wheelSpeeds.frontLeftMetersPerSecond);
                m_frontRightWheel.applyWheelSpeed(wheelSpeeds.frontRightMetersPerSecond);
                m_backLeftWheel.applyWheelSpeed(wheelSpeeds.rearLeftMetersPerSecond);
                m_backRightWheel.applyWheelSpeed(wheelSpeeds.rearRightMetersPerSecond);
        }

        private double[] desaturateVoltage(double... input) {
                double greatest = 0;
                for (int i = 0; i < input.length; i++) {
                        if (Math.abs(input[i]) > Math.abs(greatest)) {
                                greatest = input[i];
                        }
                }
                if (Math.abs(greatest) > 12) {
                        double divisor = Math.abs(greatest) / 12.0;
                        for (int i = 0; i < input.length; i++) {
                                input[i] /= divisor;
                        }
                }
                return input;
        }

        public void applyVolts(Voltage voltage) {
                m_frontLeftWheel.applyVoltage(voltage.in(Volts));
                m_backRightWheel.applyVoltage(voltage.in(Volts));
                m_frontLeftWheel.applyVoltage(voltage.in(Volts));
                m_backRightWheel.applyVoltage(voltage.in(Volts));
        }

        public Command getSysIdAuto() {
                return new SequentialCommandGroup(
                                m_sysIdRoutine.quasistatic(Direction.kForward),
                                m_sysIdRoutine.quasistatic(Direction.kReverse),
                                m_sysIdRoutine.dynamic(Direction.kForward),
                                m_sysIdRoutine.dynamic(Direction.kReverse));
        }

        public void spinFrontLeft() {
                m_frontLeftWheel.applyVoltage(12);;
        }
        public void spinFrontRight() {
                m_frontRightWheel.applyVoltage(12);;
        }
        public void spinBackLeft() {
                m_backLeftWheel.applyVoltage(12);;
        
        }public void spinBackRight() {
                m_backRightWheel.applyVoltage(12);;
        }
        public void stop() {
                m_frontLeftWheel.applyVoltage(0);
                m_frontRightWheel.applyVoltage(0);
                m_backLeftWheel.applyVoltage(0);
                m_backRightWheel.applyVoltage(0);
        }

        @Override
        public void periodic() {
                // This method will be called once per scheduler run
        }
}

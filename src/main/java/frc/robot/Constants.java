// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    public static final double kDeadzone = 0.2;
    public static final int kDriverControllerPort = 0;
    public static final LinearVelocity kNormalWheelSpeed = LinearVelocity.ofBaseUnits(1, MetersPerSecond); //change later
    public static final AngularVelocity kRobotTurnSpeed = AngularVelocity.ofBaseUnits(3, RadiansPerSecond); //change later
  }
  public static class DrivetrainConstants{
    public static final int kFrontLeftMotorID = 10;
    public static final int kFrontRightMotorID = 11;
    public static final int kBackLeftMotorID = 12;
    public static final int kBackRightMotorID = 13;

    public static final double kSlewRateLimitForMotors = 1.0;

    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final Velocity<VoltageUnit> kSysIdRampRate = null;
    public static final Voltage kStepVoltage = null;
    public static final Time kTimeout = null;

    

    public static final int kFrontLeftMotorA = 0;
    public static final int kFrontLeftMotorB = 1;
    public static final int kFrontRightMotorA = 2;
    public static final int kFrontRightMotorB = 3;
    public static final int kBackLeftMotorA = 4;
    public static final int kBackLeftMotorB = 5;
    public static final int kBackRightMotorA = 6;
    public static final int kBackRightMotorB = 7;

    public static final boolean kFrontLeftInverted = false;
    public static final boolean kFrontRightInverted = true;
    public static final boolean kBackLeftInverted = false;
    public static final boolean kBackRightInverted = true;

    public static final boolean kFrontLeftEncoderInverted = false;
    public static final boolean kFrontRightEncoderInverted = false;
    public static final boolean kBackLeftEncoderInverted = false;
    public static final boolean kBackRightEncoderInverted = false;
    
    public static final double kDistancePerPulse = 7.0/20.0; //or 20.0/7.0 we dont know  

    public static final Distance kTrackwidth = Distance.ofRelativeUnits(65.5, Centimeters);
    public static final Distance kWheelBase = Distance.ofRelativeUnits(52.5, Centimeters);

    public static final Distance kWheelDiameter = Distance.ofRelativeUnits(10, Inches);
    public static final AngularVelocity kMotorMaxAngularVelocity = AngularVelocity.ofRelativeUnits(2500 / 60.0, RotationsPerSecond); // theoretical 5310
    public static final double kMotorGearboxRatio = 1.0/9.0;

    public static final AngularVelocity kWheelMaxAngularVelocity = kMotorMaxAngularVelocity.times(kMotorGearboxRatio);

    public static final Distance kWheelCircumference = kWheelDiameter.times(Math.PI);


    public static final LinearVelocity kMaxWheelSpeed = LinearVelocity.ofBaseUnits(
      kWheelMaxAngularVelocity.in(RotationsPerSecond) * kWheelCircumference.in(Meter), MetersPerSecond); //change later

    public static final Distance kTrackWidthCircumference = kTrackwidth.times(Math.PI);
    public static final AngularVelocity kMaxRobotTurnSpeed = AngularVelocity.ofRelativeUnits(
      kTrackWidthCircumference.in(Meter) / kMaxWheelSpeed.in(MetersPerSecond), RotationsPerSecond);
  }
  public static class PneumaticConstants{
    public static final int kCannonCount = 6;
    public static final double kFiringTime = 0.2; //As seconds
  }
}

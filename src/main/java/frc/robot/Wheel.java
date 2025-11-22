package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import frc.robot.Constants.DrivetrainConstants;

public class Wheel {
    private String m_motorName;

    private WPI_TalonSRX m_motor;
    private Encoder m_encoder;

    private SlewRateLimiter m_limiter;
    private PIDController m_PID;
    private SimpleMotorFeedforward m_FF;

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    public MotorLog m_log;

    public Wheel(String motorName, int motorID, int encoderChannelA, int encoderChannelB,
            boolean isMotorInverted, boolean isEncoderInverted) {
        m_motorName = motorName;

        m_motor = new WPI_TalonSRX(motorID);
        m_motor.configAllSettings(new TalonSRXConfiguration());
        m_motor.setInverted(isMotorInverted);

        m_encoder = new Encoder(encoderChannelA, encoderChannelB, isEncoderInverted);
        m_encoder.setDistancePerPulse(DrivetrainConstants.kDistancePerPulse);

        m_limiter = new SlewRateLimiter(DrivetrainConstants.kSlewRateLimitForMotors);
        m_PID = new PIDController(DrivetrainConstants.kP,
                DrivetrainConstants.kI,
                DrivetrainConstants.kD);
        m_FF = new SimpleMotorFeedforward(DrivetrainConstants.kS,
                DrivetrainConstants.kV,
                DrivetrainConstants.kA);
    }

    public void applyWheelSpeed(double speedMetersPerSecond) {

        speedMetersPerSecond = m_limiter.calculate(speedMetersPerSecond);
        applyVoltage(getVoltageForSpeed(speedMetersPerSecond));

    }

    private double getVoltageForSpeed(double speedMetersPerSecond) {
        double voltage = m_FF.calculate(speedMetersPerSecond)
                + m_PID.calculate(getSpeedMetersPerSecond(), speedMetersPerSecond);
        return voltage;
    }

    public void applyVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    public double getSpeedMetersPerSecond() {
        double rotationsPerSecond = m_encoder.getRate();
        SmartDashboard.putNumber(m_motorName + " MPS", rotationsPerSecond);
        return rotationsPerSecond
                * DrivetrainConstants.kWheelCircumference.in(Meter);
    }

    public double getVoltage() {
        return m_motor.get() * m_motor.getBusVoltage();
    }

    public double getCurrent() {
        return m_motor.getStatorCurrent();
    }

    public double getDistanceMeters(){
        return m_encoder.getDistance() * DrivetrainConstants.kWheelCircumference.in(Meters);
    }

}

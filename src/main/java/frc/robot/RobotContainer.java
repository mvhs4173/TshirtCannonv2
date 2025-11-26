// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Pneumatics m_pneumatics = new Pneumatics();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(
        new RunCommand(() -> {
          m_drivetrain.userDrive(
              -m_driverController.getLeftY(),
              m_driverController.getLeftX(),
              m_driverController.getRightX(),
              m_driverController.getHID().getRightBumperButton());
        },
            m_drivetrain));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().onTrue(new InstantCommand(m_pneumatics::fireCannon, m_pneumatics));
    m_driverController.y().onTrue(new InstantCommand(m_drivetrain::resetGyro, m_drivetrain));
    m_driverController.povUp().whileTrue(new RunCommand(m_drivetrain::spinFrontLeft, m_drivetrain)).onFalse(new InstantCommand(m_drivetrain::stop));
    m_driverController.povDown().whileTrue(new RunCommand(m_drivetrain::spinBackRight, m_drivetrain)).onFalse(new InstantCommand(m_drivetrain::stop));
    m_driverController.povLeft().whileTrue(new RunCommand(m_drivetrain::spinBackLeft, m_drivetrain)).onFalse(new InstantCommand(m_drivetrain::stop));
    m_driverController.povRight().whileTrue(new RunCommand(m_drivetrain::spinFrontRight, m_drivetrain)).onFalse(new InstantCommand(m_drivetrain::stop));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_drivetrain.getSysIdAuto();
  }
}

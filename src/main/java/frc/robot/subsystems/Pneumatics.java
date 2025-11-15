// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class Pneumatics extends SubsystemBase {
  private Solenoid[] m_cannons = new Solenoid[PneumaticConstants.kCannonCount];
  private Compressor m_compressor;
  private int m_nextCannon = 0;

  /** Creates a new Pneumatics. */
  public Pneumatics() {
    for(int i = 0; i < PneumaticConstants.kCannonCount; i++){
      m_cannons[i] = new Solenoid(PneumaticsModuleType.CTREPCM, i);
      m_cannons[i].setPulseDuration(PneumaticConstants.kFiringTime);
    }
    m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    m_compressor.enableDigital();
  }

  /**
   * Fires the next cannon that needs to fire, and updates the new next cannon
   */
  public void fireCannon() {
    m_cannons[m_nextCannon].startPulse();
    m_nextCannon = (m_nextCannon + 1) % PneumaticConstants.kCannonCount;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

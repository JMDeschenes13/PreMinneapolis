// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;



public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private final int PCMCANid = ManipulatorConstants.PCMCANId;
  private final int kforwardChannel = ManipulatorConstants.karmExtendPort ;
  private final int kbackwardChannel = ManipulatorConstants.karmRetractPort;
  private final DoubleSolenoid m_solenoid = new DoubleSolenoid(PCMCANid,PneumaticsModuleType.CTREPCM,kforwardChannel, kbackwardChannel);

  public Arm() {}

  public void forward(){
    m_solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void backward(){
    m_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

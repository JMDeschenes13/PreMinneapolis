// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Claw extends SubsystemBase {

  private final int kforwardChannel = ManipulatorConstants.kclawOpenPort;
  private final int kbackwardChannel = ManipulatorConstants.kclawClosePort;
  private final DoubleSolenoid m_solenoid = new DoubleSolenoid(ManipulatorConstants.PCMCANId,PneumaticsModuleType.CTREPCM,kforwardChannel, kbackwardChannel);
  /** Creates a new Pnuematics. */
  public Claw() {


  }

  public void clawClose(){
    m_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void clawOpen(){
    m_solenoid.set(DoubleSolenoid.Value.kForward);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


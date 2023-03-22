// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.LightsConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;



public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  Spark lights;
  public Lights() {
    lights = new Spark(LightsConstants.kLightsPWMPort);

  }
  public void solidPurple(){
    lights.set(0.91);
  }

  public void solidYellow(){
    lights.set(0.69);
  }

  public void flashy(){
    lights.set(0.19);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

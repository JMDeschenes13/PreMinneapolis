// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class SolidPurple extends CommandBase {
  Lights m_lights;
  /** Creates a new SolidPurple. */
  public SolidPurple(Lights lights) {
    
    m_lights = lights;
    addRequirements(m_lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lights.solidPurple();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

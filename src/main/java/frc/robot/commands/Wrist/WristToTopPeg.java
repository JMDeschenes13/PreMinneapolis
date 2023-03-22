// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import frc.robot.subsystems.Wrist;
import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WristToTopPeg extends CommandBase {
  /** Creates a new WristToTopPeg. */
  private Wrist m_wrist;
  private double targetPosition = ManipulatorConstants.ktopPegEncoderPosition;

  public WristToTopPeg(Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
    m_wrist = wrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.goTo(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wrist.atPosition(targetPosition);
  }
}

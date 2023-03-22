// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class DoTimeOut extends CommandBase {
  double endTime;
  double timeout;
  /** Creates a new DoTimeOut. */
  public DoTimeOut(double seconds) {
    timeout = seconds;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  private void startTimer(){
    endTime = Timer.getFPGATimestamp() + timeout;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= endTime;
  }
}

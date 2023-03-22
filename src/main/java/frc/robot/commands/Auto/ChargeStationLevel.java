// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;;



public class ChargeStationLevel extends CommandBase {
  boolean stop;
  DriveSubsystem m_DriveSubsystem;
  PIDController m_PIDController = new PIDController(.006,0.00,0);
  /** Creates a new ChargeStationLevel. */
  public ChargeStationLevel(DriveSubsystem DriveSubsystem, boolean stop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.stop = stop;
    m_DriveSubsystem = DriveSubsystem;
    m_PIDController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubsystem.drive(m_PIDController.calculate(m_DriveSubsystem.getPitch()),0,0,false, true);
    if(Math.abs(m_DriveSubsystem.getPitch()) < .75){
      m_PIDController.reset();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(stop){
        return Math.abs(m_DriveSubsystem.getPitch()) < 1;
    }
    else{
      return false;
    }
    
    
  }
}

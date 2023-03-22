// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.enums.AutoBalanceDirection;

public class PrototypeChargeLeveling extends CommandBase {
  DriveSubsystem m_DriveSubsystem;
  PIDController m_PIDController;
  boolean isLeveled;
  double curTime; 
  AutoBalanceDirection direction;
  double targetTime;
  /** Creates a new ProtoypeChargLeveling. */
  public PrototypeChargeLeveling(DriveSubsystem DriveSubsystem, AutoBalanceDirection direction) {
    this.direction = direction;
    m_DriveSubsystem = DriveSubsystem;
    m_PIDController = new PIDController(.008, 0, 0);
    m_PIDController.setSetpoint(0);
    isLeveled = false;
    targetTime = 0;
    
  

    addRequirements(m_DriveSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (direction){
      case FORWARD:
        m_DriveSubsystem.drive(-0.4, 0, 0, false, true);
        break;
      case BACKWARD:
        m_DriveSubsystem.drive(.4, 0,0, false, true);
        break;
    }
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle  = Math.abs(m_DriveSubsystem.getPitch());
    double stopAngle = 5;
    double levelAngle = 1;
    boolean stoppedOnce = false;
    
    if(angle >= stopAngle){
      m_DriveSubsystem.drive(m_PIDController.calculate(angle),0,0,false, true);
      curTime = Timer.getFPGATimestamp();
      targetTime = curTime + 1;
      stoppedOnce = true;
      }
    else if(stoppedOnce){
      m_DriveSubsystem.setX();
      stoppedOnce = false;
      if( targetTime <= Timer.getFPGATimestamp() && angle <= levelAngle){
        isLeveled = true;
      }

    }
      

    
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setX();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isLeveled;
  }
}

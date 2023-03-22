// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.ManipulatorConstants;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  
  private final WPI_TalonFX wristFollower;
  private final WPI_TalonFX wristMotor;
  static double iaccum = 0;
  private double sensorPosition;
  private double sensorVelocity;
 
 
 
public Wrist() {
  wristFollower = new WPI_TalonFX(ManipulatorConstants.kwristFollowerMotor);
   wristMotor = new WPI_TalonFX(ManipulatorConstants.kwristMotor);
   // Configures the Intake Victors's to default configuration
   wristMotor.configFactoryDefault();
   wristFollower.configFactoryDefault();
     
   // Invert motor direction if set to true
   wristMotor.setInverted(false);
   wristFollower.setInverted(true);

  

   //Set Motors to coast mode
   wristMotor.setNeutralMode(NeutralMode.Brake);
   wristFollower.setNeutralMode(NeutralMode.Brake);

  wristFollower.follow(wristMotor);

   

  

   /*___________________________________________________
      
                  Flywheel ENCODER SET UP
      ____________________________________________________*/

    /* first choose the sensor */
     
    wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);

    wristMotor.setSensorPhase(true);

    // Reset sensor position
    wristMotor.setIntegralAccumulator(iaccum, 0, 10);

    wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, EncoderConstants.kTimeoutMs);

    wristMotor.configAllowableClosedloopError(0, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);
  
    /* set the peak and nominal outputs */
     
    wristMotor.configNominalOutputForward(0, EncoderConstants.kTimeoutMs);
    wristMotor.configNominalOutputReverse(0, EncoderConstants.kTimeoutMs);
    wristMotor.configPeakOutputForward(ManipulatorConstants.kwristMaxOutputPercentage, EncoderConstants.kTimeoutMs);
    wristMotor.configPeakOutputReverse(-ManipulatorConstants.kwristMaxOutputPercentage, EncoderConstants.kTimeoutMs);
  
    /* set closed loop gains in slot0 - see documentation */
    
    wristMotor.config_kF(0, 0.0, EncoderConstants.kTimeoutMs);
    wristMotor.config_kP(0, 0.35, EncoderConstants.kTimeoutMs);
    wristMotor.config_kI(0, 0.00055, EncoderConstants.kTimeoutMs);
    wristMotor.config_kD(0, 1.0, EncoderConstants.kTimeoutMs); 

    /* zero the sensor */
    wristMotor.setSelectedSensorPosition(0, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);

    sensorPosition = wristMotor.getSelectedSensorPosition();

    wristMotor.configMotionAcceleration(ManipulatorConstants.kwristAcceleration);
    wristMotor.configMotionCruiseVelocity(ManipulatorConstants.kwristSpeed);





    //WRISTFOLLOWERMOTOR

    wristFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);

    wristFollower.setSensorPhase(true);

    // Reset sensor position
    wristFollower.setIntegralAccumulator(iaccum, 0, 10);

    wristFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, EncoderConstants.kTimeoutMs);

    wristFollower.configAllowableClosedloopError(0, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);
  
    /* set the peak and nominal outputs */
     
    wristFollower.configNominalOutputForward(0, EncoderConstants.kTimeoutMs);
    wristFollower.configNominalOutputReverse(0, EncoderConstants.kTimeoutMs);
    wristFollower.configPeakOutputForward(ManipulatorConstants.kwristMaxOutputPercentage, EncoderConstants.kTimeoutMs);
    wristFollower.configPeakOutputReverse(-ManipulatorConstants.kwristMaxOutputPercentage, EncoderConstants.kTimeoutMs);
  
    /* set closed loop gains in slot0 - see documentation */
    
    wristFollower.config_kF(0, 0.0, EncoderConstants.kTimeoutMs);
    wristFollower.config_kP(0, 0.35, EncoderConstants.kTimeoutMs);
    wristFollower.config_kI(0, 0.00055, EncoderConstants.kTimeoutMs);
    wristFollower.config_kD(0, 1.0, EncoderConstants.kTimeoutMs); 

    /* zero the sensor */
    wristFollower.setSelectedSensorPosition(0, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);

    sensorPosition = wristFollower.getSelectedSensorPosition();

    wristFollower.configMotionAcceleration(ManipulatorConstants.kwristAcceleration);
    wristFollower.configMotionCruiseVelocity(ManipulatorConstants.kwristSpeed);

    



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sensorPosition = wristMotor.getSelectedSensorPosition();
    //sensorVelocity = wristMotor.getSelectedSensorVelocity();

     //posError = sensorPosition - dCurrentPosition;

     //pushes values to the dashboard
     SmartDashboard.putNumber("WristSensorVelocity", sensorVelocity);
     SmartDashboard.putNumber("WristSensorPosition", sensorPosition);
     SmartDashboard.putNumber("WristTargetPosition", 0);
     //SmartDashboard.putNumber("FlipperOutputPercent", wristMotor.getMotorOutputPercent());
     //SmartDashboard.putNumber("FlipperTargetPosition", targetPosition);
     //SmartDashboard.putNumber("FipperNewTarget", CurrentPosition);
     //SmartDashboard.putNumber("FlipperPosError", posError);
  }

  public void WristToStart(){
    wristMotor.set(ControlMode.MotionMagic, ManipulatorConstants.kstartEncoderPosition);

  }
  public ArrayList<TalonFX> getMotors(){
    ArrayList<TalonFX> listOfMotors = new ArrayList<>();
    listOfMotors.add(wristMotor);
    listOfMotors.add(wristFollower);
    return listOfMotors;
  }

  public void goTo(double targetEncoderPosition){


    wristMotor.set(ControlMode.MotionMagic, targetEncoderPosition);
    SmartDashboard.putNumber("WristMotor1Current", wristMotor.getSupplyCurrent());
    SmartDashboard.putNumber("WristMotor2Current", wristFollower.getSupplyCurrent());
    SmartDashboard.putNumber("WristTargetPosition", targetEncoderPosition);

}

  public boolean atPosition(double targetEncoderPosition){
    if (Math.abs(wristMotor.getSelectedSensorPosition() - targetEncoderPosition) <= ManipulatorConstants.kwristEncoderThreshold){
      return true;
    }
    else{
      return false;
    }
  }
}

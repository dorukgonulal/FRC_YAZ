// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotID;

public class PivotSubsystem extends SubsystemBase {

  double positionZero = 0;
  private SparkMax PivotMotor = new SparkMax(RobotID.Pivot.MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder pivotEncoder = PivotMotor.getEncoder(); 

  public final PIDController pivotController;

  private double PIDOutput = 0;

  public enum PivotPosition{
    CLOSE, REEFL23, REEFL4, HP, NORMAL, REEFL1, PROCESSOR, BARGE
  }

  public PivotSubsystem() {
    pivotController = new PIDController(Constants.PivotConstants.PIVOT_KP, Constants.PivotConstants.PIVOT_KI, Constants.PivotConstants.PIVOT_KD);
    pivotController.setIntegratorRange(-0.5, 0.5); //TODO: This must be tuned
    pivotController.setTolerance(0.5, 6); //TODO: This must be tuned
    pivotEncoder.setPosition(0);
    
  }

  public void setPivot(double speed){
    PivotMotor.set(speed);
  }


  public void setSetPoint(double setPoint){
    pivotController.setSetpoint(setPoint);

  }


  public void setDistance(double setPoint){
    PIDOutput = pivotController.calculate(getEncoderPosition() - setPoint);
    setPivot(PIDOutput);

  }

  public void pivotUp(double speed){
    PivotMotor.set(speed);

  }

  public void pivotDown(double speed){
    PivotMotor.set(-speed);
    
  }

  public void pivotStop(){
    PivotMotor.stopMotor();
    //PivotMotor.setIdleMode(IdleMode.kBrake);

  }

  public double getEncoderPosition(){
    return pivotEncoder.getPosition() - positionZero;

  }

  public double getPivotPosition(){
    return pivotEncoder.getPosition();

  }

  public void resetEncoder(){
    pivotEncoder.setPosition(0);

  }

  public void stallPivot(){
    PivotMotor.set(0.06);
  }

  


  @Override
  public void periodic() {
    SmartDashboard.putNumber("PIVOT | CORAL", getPivotPosition());
  }
}
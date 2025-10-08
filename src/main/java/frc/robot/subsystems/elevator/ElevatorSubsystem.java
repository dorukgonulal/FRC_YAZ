// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotID;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX leftMotor = new TalonFX(14);
    private TalonFX rightMotor = new TalonFX(15);

  public PIDController elevatorController = new PIDController(0.05, 0, 0.0);

  private double PIDOutput = 0;
  private double positionZero = 0;
  public int currentSetpoint;

  public enum ElevatorPositions {
    BASE, HP, L1, L2, L3, L4, MAX, A1, A2, PROCESSOR, AUTOL3, AUTOL2, READYFORAUTO
  }

  public ElevatorSubsystem() {
    // Configure Kraken motors
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    //rightMotor.setNeutralMode(NeutralModeValue.Brake);
    //leftMotor.setNeutralMode(NeutralModeValue.Brake);

    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    //leftMotor.getConfigurator().apply(configs);
    //rightMotor.getConfigurator().apply(configs);
    elevatorController.setTolerance(0.15);
    rightMotor.setPosition(0);
    leftMotor.setPosition(0);

  }

  public void setMotors(double speed) {
    //leftMotor.setControl(new DutyCycleOut(speed)); // Convert to voltage
    leftMotor.set(-speed);
    rightMotor.set(-speed);
  }

  public void setSetPoint(double setPoint) {
    elevatorController.setSetpoint(setPoint);
  }

  public double getSetPoint() {
    return elevatorController.getSetpoint();
  }

  public double getPidValue(double setpoint) {
    return elevatorController.calculate(getElevatorPosition(), setpoint);
  }

  public void pidReset() {
    elevatorController.reset();
  }

  public void stallElevator(double speed) {
    leftMotor.set(-speed);
    rightMotor.set(-speed);
  }

  public void setIdleMode(NeutralModeValue mode) {
    leftMotor.setNeutralMode(mode);
    rightMotor.setNeutralMode(mode);
  }

  public double getElevatorPosition() {
    return -1 * leftMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorVelocity() {
    return leftMotor.getVelocity().getValueAsDouble();
  }

  public void resetElevatorEncoder() {
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }

  public void setElevatorToPose(double pose) {
    leftMotor.setPosition(pose);
    rightMotor.setPosition(pose);
  }

  public void setDistance(double setpoint) {
    PIDOutput = elevatorController.calculate(getElevatorPosition(), setpoint);
    setMotors(PIDOutput);
  }

  public boolean atSetpoint() {
    return elevatorController.atSetpoint();
  }

  public double getError() {
    return elevatorController.getPositionError();
  }

  // Encoder Methods
  public void resetEncoder() {
    positionZero = getElevatorPosition();
  }

  public double getEncoderPosition() {
    return getElevatorPosition() - positionZero;
  }

  public double getEncoderPositionZero() {
    return positionZero;
  }

  @Override
  public void periodic() {    
    SmartDashboard.putNumber("POSITION | ELEVATOR", getElevatorPosition());

    if (getElevatorPosition() < 1) {
      stallElevator(0);
    }
    
  }
}
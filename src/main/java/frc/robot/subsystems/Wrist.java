// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  // Motors
  public CANSparkMax wristMotor = new CANSparkMax(11, MotorType.kBrushless);
  
  // Gets PID Controller and Encoder for wrist
  private SparkMaxPIDController pidController = wristMotor.getPIDController();
  public RelativeEncoder wristEncoder = wristMotor.getEncoder();

  // Creates a new Wrist.
  public Wrist() {
    setup();
    setSmartMotionParams();
    setPid();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setup() {
    wristMotor.restoreFactoryDefaults();
    wristMotor.clearFaults();

    wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.WristConstants.SoftLimits.forwardSoftLimit);
    wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.WristConstants.SoftLimits.reverseSoftLimit);

    wristMotor.setInverted(false);
    
    pidController.setFeedbackDevice(wristEncoder);
    pidController.setOutputRange(-1.0, 1.0);
  }

  private void setPid() {
    pidController.setIZone(Constants.WristConstants.PID.iZone);
    pidController.setP(Constants.WristConstants.PID.kP);
    pidController.setI(Constants.WristConstants.PID.kI);
    pidController.setD(Constants.WristConstants.PID.kD);
    pidController.setFF(Constants.WristConstants.PID.fGain);
  }

  private void setSmartMotionParams() {
    pidController.setSmartMotionMaxVelocity(Constants.WristConstants.SmartMotionParameters.maxVel, Constants.WristConstants.SmartMotionParameters.smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(Constants.WristConstants.SmartMotionParameters.minVel, Constants.WristConstants.SmartMotionParameters.smartMotionSlot);
    pidController.setSmartMotionMaxAccel(Constants.WristConstants.SmartMotionParameters.maxAccel, Constants.WristConstants.SmartMotionParameters.smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(Constants.WristConstants.SmartMotionParameters.maxErr, Constants.WristConstants.SmartMotionParameters.smartMotionSlot);
  }

  public double getCurrentPosition() {
    return this.wristEncoder.getPosition();
  }

  public void setPosition(double targetPosition) {
    pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setSmartMotionPosition(double targetPosition) {
    pidController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  public double getWristCurrent() {
    return wristMotor.getOutputCurrent();
  }

  public void setWristPO(double voltage) {
    wristMotor.set(voltage);
  }

  public double getWristMotorVelocity() {
    return wristEncoder.getVelocity();
  }

}

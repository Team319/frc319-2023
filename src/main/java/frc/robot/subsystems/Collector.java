// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  
  public CANSparkMax collectorMotor = new CANSparkMax(12, MotorType.kBrushless);

  private SparkMaxPIDController pidController = collectorMotor.getPIDController();
  public RelativeEncoder collectorEncoder = collectorMotor.getEncoder();

  private boolean holdingCube = false;
  
  /** Creates a new Collector. */
  public Collector() {
    collectorSetUp();
  }

  public void setCollectorPO(double voltage) {
    collectorMotor.set(voltage);
  }

  public double getCollectorCurrent() {
    return collectorMotor.getOutputCurrent();
  }

  private void collectorSetUp() {
    collectorMotor.restoreFactoryDefaults();
    collectorMotor.clearFaults();

    collectorMotor.setInverted(true);

    //pidController.setFeedbackDevice(collectorEncoder);
    pidController.setFF(Constants.CollectorConstants.PID.fGain);
    pidController.setP(Constants.CollectorConstants.PID.kP);
    pidController.setOutputRange(-1, 1);

    collectorMotor.setClosedLoopRampRate(0.125);
    collectorMotor.setOpenLoopRampRate(0.125);
    
    collectorMotor.setSmartCurrentLimit(Constants.CollectorConstants.Currents.currentMax);
  }

  public void setCollectorVelocity(double velocity) {
    pidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setCollectorCurrentLimit(int currentLimit) {
    collectorMotor.setSmartCurrentLimit(currentLimit);
  }

  public double getCollectorVelocity() {
    return collectorEncoder.getVelocity();
  }

  public boolean getHoldingCube() {
    return holdingCube;
  }

  public void setHoldingCube(boolean holdingCube) {
    this.holdingCube = holdingCube;
  }
}

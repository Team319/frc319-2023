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

public class Wrist extends SubsystemBase {

  //Motors
  public CANSparkMax wristPivot = new CANSparkMax(10, MotorType.kBrushless);
  
  public RelativeEncoder wristEncoder = wristPivot.getEncoder();
  private SparkMaxPIDController pidController = wristPivot.getPIDController();

  // Creates a new Wrist.
  public Wrist() {
    setPid();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCurrentPosition() {
    return this.wristEncoder.getPosition();
  }

  public void setPosition(double targetPosition) {
    pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  private void setPid() {
    pidController.setIZone(Constants.WristConstants.PID.iZone);
    pidController.setP(Constants.WristConstants.PID.kP);
    pidController.setI(Constants.WristConstants.PID.kI);
    pidController.setD(Constants.WristConstants.PID.kD);
    pidController.setFF(Constants.WristConstants.PID.fGain);
  }
}

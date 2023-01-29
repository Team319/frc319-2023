// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  //Motors
  public CANSparkMax elevatorLead = new CANSparkMax(8, MotorType.kBrushless);
  public CANSparkMax elevatorFollow = new CANSparkMax(9, MotorType.kBrushless);
  
  public RelativeEncoder elevatorEncoder = elevatorLead.getEncoder();
  private SparkMaxPIDController pidController = elevatorLead.getPIDController();

  /** Creates a new Elevator. */
  public Elevator() {
    pidController.setIZone(Constants.ElevatorConstants.PID.iZone);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCurrentPosition() {
    return this.elevatorEncoder.getPosition();
  }

  public void setPosition(double targetPosition) {
    manageMotion(targetPosition);
    pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  private void pidUp() {
    pidController.setP(Constants.ElevatorConstants.PID.kPUp);
    pidController.setI(Constants.ElevatorConstants.PID.kIUp);
    pidController.setD(Constants.ElevatorConstants.PID.kDUp);
    pidController.setFF(Constants.ElevatorConstants.PID.fGainUp);
  }

  private void pidDown() {
    pidController.setP(Constants.ElevatorConstants.PID.kPDown);
    pidController.setI(Constants.ElevatorConstants.PID.kIDown);
    pidController.setD(Constants.ElevatorConstants.PID.kDDown);
    pidController.setFF(Constants.ElevatorConstants.PID.fGainDown);
  }
  
  private void manageMotion(double targetPosition) {
    double currentPosition = getCurrentPosition();
      if (currentPosition < targetPosition) {
        pidUp();
      } else {
        pidDown();
      }
  }
}

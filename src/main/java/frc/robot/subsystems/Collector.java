// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  
  public CANSparkMax collectorMotor = new CANSparkMax(12, MotorType.kBrushless);
  /** Creates a new Collector. */
  public Collector() {
    collectorSetUp();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCollectorPO(double voltage) {
    collectorMotor.set(voltage);
  }

  public double getCollectorCurrent() {
    return collectorMotor.getOutputCurrent();
  }

  public void collectorSetUp() {
    collectorMotor.restoreFactoryDefaults();
    collectorMotor.setInverted(true);

    collectorMotor.setSmartCurrentLimit(Constants.CollectorConstants.Currents.currentMax);
  }
}

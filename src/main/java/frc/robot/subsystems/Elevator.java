// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  //Motors
  public CANSparkMax elevatorLead = new CANSparkMax(8, MotorType.kBrushless);
  public CANSparkMax elevatorFollow = new CANSparkMax(9, MotorType.kBrushless);

  /** Creates a new Elevator. */
  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*public void manageMotion(double targetPosition) {
    double currentPosition = getCurrentPosition();
    if (Robot.mode == RobotMode.Climb) {
      this.elevatorLead.configPeakOutputReverse(-0.5);
      if (currentPosition < targetPosition) {
        elevatorLead.selectMotionParameters(climbUpMotionParameters);
      } else {
        elevatorLead.selectMotionParameters(climbDownMotionParameters);
      }
    } else {
      this.elevatorLead.configPeakOutputReverse(-1.0);
      if (currentPosition < targetPosition) {
        elevatorLead.selectMotionParameters(upMotionParameters);
      } else {
        elevatorLead.selectMotionParameters(downMotionParameters);
      }
    }

  }*/ // Copied from 2019 Code - L.S
}

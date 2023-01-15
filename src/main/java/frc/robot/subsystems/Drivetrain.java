// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.BobDrive;
import frc.robot.utils.DriveSignal;

public class Drivetrain extends SubsystemBase {

  // Motors
  public TalonFX leftLead = new TalonFX(2);
  public TalonFX leftFollow1 = new TalonFX(1);
  public TalonFX leftFollow2 = new TalonFX(3);

  public TalonFX rightLead = new TalonFX(5);
  public TalonFX rightFollow1 = new TalonFX(4);
  public TalonFX rightFollow2 = new TalonFX(6);


  /** Creates a new Drivetrain. */
  public Drivetrain() {

    leftLead.setInverted(true);
    leftFollow1.setInverted(true);
    leftFollow2.setInverted(true);

    rightLead.setInverted(false);
    rightFollow1.setInverted(false);
    rightFollow2.setInverted(false);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorOutput(double leftVoltage, double rightVoltage) {
    leftLead.set(TalonFXControlMode.PercentOutput, 0.5*leftVoltage);
    rightLead.set(TalonFXControlMode.PercentOutput, 0.5*rightVoltage);

  }

  public void drive(ControlMode controlMode, double left, double right) {
    //Left control mode is set to left
    this.leftLead.set(controlMode, right);

    //Right control mode is set to right
    this.rightLead.set(controlMode, right);
  }
  
  public void drive(ControlMode controlMode, DriveSignal driveSignal) {
    this.drive(controlMode, driveSignal.getLeft(), driveSignal.getRight());
  }

  public void setFollowers() {
    leftFollow1.follow(leftLead);
    leftFollow2.follow(leftLead);

    rightFollow1.follow(rightLead);
    rightFollow2.follow(rightLead);
  }
}


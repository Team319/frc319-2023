// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elbow extends SubsystemBase {
  
  public CANSparkMax elbowMotor = new CANSparkMax(10, MotorType.kBrushless);

  private SparkMaxPIDController pidController = elbowMotor.getPIDController();
  public SparkMaxAlternateEncoder.Type elbowEncoder = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 8192;
  private RelativeEncoder m_AlternateEncoder;

  /** Creates a new Elbow. */
  public Elbow() {
    setup();
    setUpPid();
    setDownPid();
    setSmartMotionParams();
  }

  /**
   * Chooses the PID profile to use based on current position and target position
   * 
   * @param targetPosition position that we want to go to
   */
  private void manageMotion(double targetPosition) {
    double currentPosition = getCurrentPosition();
      if (currentPosition < targetPosition) {
        setUpPid();
      } else {
        setDownPid();
      }
  }

  /**
   * Sets the up PID values
   */
  private void setUpPid() {
    pidController.setIZone(Constants.ElbowConstants.PID.iZone);
    pidController.setP(Constants.ElbowConstants.PID.kUpP);
    pidController.setI(Constants.ElbowConstants.PID.kUpI);
    pidController.setD(Constants.ElbowConstants.PID.kUpD);
    pidController.setFF(Constants.ElbowConstants.PID.fGain);
  }

  /**
   * Sets the down PID values
   */
  private void setDownPid() {
    pidController.setIZone(Constants.ElbowConstants.PID.iZone);
    pidController.setP(Constants.ElbowConstants.PID.kDownP);
    pidController.setI(Constants.ElbowConstants.PID.kDownI);
    pidController.setD(Constants.ElbowConstants.PID.kDownD);
    pidController.setFF(Constants.ElbowConstants.PID.fGain);
  }
  private void setSmartMotionParams() {
    pidController.setSmartMotionMaxVelocity(Constants.ElbowConstants.SmartMotionParameters.maxVel, Constants.ElbowConstants.SmartMotionParameters.smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(Constants.ElbowConstants.SmartMotionParameters.minVel, Constants.ElbowConstants.SmartMotionParameters.smartMotionSlot);
    pidController.setSmartMotionMaxAccel(Constants.ElbowConstants.SmartMotionParameters.maxAccel, Constants.ElbowConstants.SmartMotionParameters.smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(Constants.ElbowConstants.SmartMotionParameters.maxErr, Constants.ElbowConstants.SmartMotionParameters.smartMotionSlot);
  }

  private void setup() {
    elbowMotor.restoreFactoryDefaults();
    elbowMotor.clearFaults();

    elbowMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    elbowMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    elbowMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ElbowConstants.SoftLimits.forwardSoftLimit);
    elbowMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ElbowConstants.SoftLimits.reverseSoftLimit);

    elbowMotor.setInverted(true);

    m_AlternateEncoder = elbowMotor.getAlternateEncoder(elbowEncoder, kCPR);
    
    pidController.setFeedbackDevice(m_AlternateEncoder);
    pidController.setOutputRange(-1.0, 1.0);

    elbowMotor.setSmartCurrentLimit(30);
  }

  /**
   * Gets the current position of the elbow encoder
   * 
   * @return returns position of the elbow encoder
   */
  public double getCurrentPosition() {
    return this.m_AlternateEncoder.getPosition();
  }

  public void setPosition(double targetPosition) {
    manageMotion(targetPosition);
    pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setSmartMotionPosition(double targetPosition) {
    manageMotion(targetPosition);
    pidController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  public double getElbowCurrent() {
    return elbowMotor.getOutputCurrent();
  }

  public void setElbowPO(double voltage) {
    elbowMotor.set(voltage);
  }

  public double getElbowMotorVelocity() {
    return m_AlternateEncoder.getVelocity();
  }
}

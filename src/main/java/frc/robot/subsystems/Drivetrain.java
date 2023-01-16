// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
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

  public PigeonIMU pigeon = new PigeonIMU(7);

  private DifferentialDriveOdometry odometry;
  Rotation2d heading = new Rotation2d(Units.degreesToRadians(pigeon.getFusedHeading()));

  private static final double rampRate = 0.25;

  //DifferentialDrive differentialDrive = new DifferentialDrive(leftLead, rightLead);

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    setMotorConfigsToDefault();
    setMotorInversions();
    setMotorNeutralModes();
    setMotorRampRates();
    

    //TODO reset encoders
    odometry = new DifferentialDriveOdometry(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters());

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters());
  }

  public void drive(ControlMode controlMode, double left, double right) {
    //Left control mode is set to left
    this.leftLead.set(controlMode, left);

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

  private void setMotorConfigsToDefault() {
    
    leftLead.configFactoryDefault();
    leftFollow1.configFactoryDefault();
    leftFollow2.configFactoryDefault();
    
    rightLead.configFactoryDefault();
    rightFollow1.configFactoryDefault();
    rightFollow2.configFactoryDefault();
  }

  private void setMotorInversions() {
    
    leftLead.setInverted(false);
    leftFollow1.setInverted(false);
    leftFollow2.setInverted(false);

    rightLead.setInverted(true);
    rightFollow1.setInverted(true);
    rightFollow2.setInverted(true);
  }

  private void setMotorNeutralModes() {
    
    leftLead.setNeutralMode(NeutralMode.Coast);
    leftFollow1.setNeutralMode(NeutralMode.Coast);
    leftFollow2.setNeutralMode(NeutralMode.Coast);

    rightLead.setNeutralMode(NeutralMode.Coast);
    rightFollow1.setNeutralMode(NeutralMode.Coast);
    rightFollow2.setNeutralMode(NeutralMode.Coast);
  }

  private void setMotorRampRates() {

    leftLead.configOpenloopRamp(rampRate);
    leftFollow1.configOpenloopRamp(rampRate);
    leftFollow2.configOpenloopRamp(rampRate);

    rightLead.configOpenloopRamp(rampRate);
    rightFollow1.configOpenloopRamp(rampRate);
    rightFollow2.configOpenloopRamp(rampRate);
  }

  public double getLeftLeadDriveDistanceMeters() {
    return this.leftLead.getSelectedSensorPosition() * Constants.DriveConstants.metersPerEncoderTick;
  }

  public double getRightLeadDriveDistanceMeters() {
    return this.rightLead.getSelectedSensorPosition() * Constants.DriveConstants.metersPerEncoderTick;
  }

  public double getLeftLeadDriveDistanceTicks() {
    return this.leftLead.getSelectedSensorPosition();
  }

  public double getRightLeadDriveDistanceTicks() {
    return this.rightLead.getSelectedSensorPosition();
  }

  public Rotation2d getHeading() {
    heading = new Rotation2d(Units.degreesToRadians(pigeon.getFusedHeading()));
    return heading;
  }

  public void resetHeading() {
    pigeon.setFusedHeading(0);
    this.getHeading();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() { // TODO : These should be scaled with the distance travelled per 'pulse'
    	return new DifferentialDriveWheelSpeeds(leftLead.getSelectedSensorVelocity(), rightLead.getSelectedSensorVelocity());
  }

  public void zeroOdometry() {
    resetEncoders();
    resetHeading();

    Pose2d origin = new Pose2d(0,0,new Rotation2d(0));
    odometry.resetPosition(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters(), origin);
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters(), pose);
  }

  public void resetEncoders() {
    
    leftLead.setSelectedSensorPosition(0);
    leftFollow1.setSelectedSensorPosition(0);
    leftFollow2.setSelectedSensorPosition(0);

    rightLead.setSelectedSensorPosition(0);
    rightFollow1.setSelectedSensorPosition(0);
    rightFollow2.setSelectedSensorPosition(0);
  }

  public void tankDriveVolts(double leftVoltage, double rightVoltage) {
    leftLead.set(TalonFXControlMode.PercentOutput, leftVoltage);
    rightLead.set(TalonFXControlMode.PercentOutput, rightVoltage);
  }

}


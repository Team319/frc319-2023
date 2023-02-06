// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Acceleration value is 33% faster than our velocity value
// TODO: Create "pigeon" to "pigeon2"

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.BobDrive;
import frc.robot.utils.DriveSignal;

public class Drivetrain extends SubsystemBase {

  private DriveMode drivemode = Constants.DriveConstants.DriveMode.Normal;

  // Motors
  public TalonFX leftLead = new TalonFX(2);
  public TalonFX leftFollow1 = new TalonFX(1);
  public TalonFX leftFollow2 = new TalonFX(3);

  public TalonFX rightLead = new TalonFX(5);
  public TalonFX rightFollow1 = new TalonFX(4);
  public TalonFX rightFollow2 = new TalonFX(6);

  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(7);

  private DifferentialDriveOdometry odometry;
  Rotation2d heading = new Rotation2d(Units.degreesToRadians(pigeon.getAngle()));
  // TODO: Check polarity

  private static final double rampRate = 0.25;
  private static double previousPitch = 0.0;
  private static double deltaPitch = 0.0;

  //DifferentialDrive differentialDrive = new DifferentialDrive(leftLead, rightLead);

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    setMotorConfigsToDefault();
    setMotorInversions();
    setDefaultMotorNeutralModes();
    setMotorRampRates();
    

    //TODO reset encoders
    odometry = new DifferentialDriveOdometry(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters());

  }

  public void initDefaultCommand() {
    setDefaultCommand(new BobDrive());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(), getLeftLeadDriveDistanceMeters(), getRightLeadDriveDistanceMeters());
    updateDeltaPitch();
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

  // Unsure if we should remove this. - L.S
  private void setDefaultMotorNeutralModes() {
    
    leftLead.setNeutralMode(NeutralMode.Coast);
    leftFollow1.setNeutralMode(NeutralMode.Coast);
    leftFollow2.setNeutralMode(NeutralMode.Coast);

    rightLead.setNeutralMode(NeutralMode.Coast);
    rightFollow1.setNeutralMode(NeutralMode.Coast);
    rightFollow2.setNeutralMode(NeutralMode.Coast);
  }

  public void setNeutralMode(NeutralMode neutralMode) {
		this.leftLead.setNeutralMode(neutralMode);
		this.rightLead.setNeutralMode(neutralMode);
	}


  private void setMotorRampRates() {

    leftLead.configOpenloopRamp(rampRate);
    leftFollow1.configOpenloopRamp(rampRate);
    leftFollow2.configOpenloopRamp(rampRate);

    rightLead.configOpenloopRamp(rampRate);
    rightFollow1.configOpenloopRamp(rampRate);
    rightFollow2.configOpenloopRamp(rampRate);
  }

  // Sets left and right controlmodes to left and right
  public void drive(ControlMode controlMode, double left, double right) {
    this.leftLead.set(controlMode, left);
    this.rightLead.set(controlMode, right);
  }
  
  // Getting the drive signals to get the left and right signals
  public void drive(ControlMode controlMode, DriveSignal driveSignal) {
    this.drive(controlMode, driveSignal.getLeft(), driveSignal.getRight());
  }

  public void tankDriveVolts(double leftVoltage, double rightVoltage) {
    leftLead.set(TalonFXControlMode.PercentOutput, leftVoltage);
    rightLead.set(TalonFXControlMode.PercentOutput, rightVoltage);
  }

  // Sets followers to lead
  public void setFollowers() {
    leftFollow1.follow(leftLead);
    leftFollow2.follow(leftLead);

    rightFollow1.follow(rightLead);
    rightFollow2.follow(rightLead);
  }

  // Gets left and right lead motorspeeds
  public double getLeftMotorSpeed() {
    double leftmotorspeed = leftLead.getSelectedSensorVelocity() * Constants.DriveConstants.metersPerEncoderTick;
    return leftmotorspeed;
  }

  public double getRightMotorSpeed() {
    double rightmotorspeed = rightLead.getSelectedSensorVelocity() * Constants.DriveConstants.metersPerEncoderTick;
    return rightmotorspeed;
  }

  // Get left and right lead driving distance in meters
  public double getLeftLeadDriveDistanceMeters() {
    return this.leftLead.getSelectedSensorPosition() * Constants.DriveConstants.metersPerEncoderTick;
  }

  public double getRightLeadDriveDistanceMeters() {
    return this.rightLead.getSelectedSensorPosition() * Constants.DriveConstants.metersPerEncoderTick;
  }

  // Get left lead and right lead driving distance in tick
  public double getLeftLeadDriveDistanceTicks() {
    return this.leftLead.getSelectedSensorPosition();
  }

  public double getRightLeadDriveDistanceTicks() {
    return this.rightLead.getSelectedSensorPosition();
  }

  public Rotation2d getHeading() {
    heading = Rotation2d.fromDegrees(MathUtil.inputModulus(pigeon.getRotation2d().getDegrees(), -180, 180));
    return heading;
  }

  public double getHeadingDegrees() {
    return MathUtil.inputModulus(pigeon.getRotation2d().getDegrees(), -180, 180);
  }

  public void resetHeading() {
    pigeon.reset();
    this.getHeading();
  }

  public double getPitch() {
    return pigeon.getPitch();
  }

  public double getDeltaPitch() {
    return deltaPitch;
  }

  public void updateDeltaPitch() {
    double currentPitch = getPitch();
    double changeOfPitch = previousPitch - currentPitch;
    previousPitch = currentPitch;
    deltaPitch = Math.abs(changeOfPitch);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() { // TODO : These should be scaled with the distance travelled per 'pulse'
    	return new DifferentialDriveWheelSpeeds(getLeftMotorSpeed(), getRightMotorSpeed());
  }

  public DriveMode getDriveMode() {
    return this.drivemode;
  }

  public void setDriveMode(DriveMode d) {
    this.drivemode = d;
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

  // Resets the encoders
  public void resetEncoders() {
    
    leftLead.setSelectedSensorPosition(0);
    leftFollow1.setSelectedSensorPosition(0);
    leftFollow2.setSelectedSensorPosition(0);

    rightLead.setSelectedSensorPosition(0);
    rightFollow1.setSelectedSensorPosition(0);
    rightFollow2.setSelectedSensorPosition(0);
  }

}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TargetType;

public class Limelight extends SubsystemBase {

  private static double fovX = 54.0;
  private static double fovY = 41.0;

  public NetworkTable defaultTable = NetworkTableInstance.getDefault().getTable("limelight-score");
  public NetworkTableEntry tx = defaultTable.getEntry("tx");
  public NetworkTableEntry ty = defaultTable.getEntry("ty");
  public NetworkTableEntry tv = defaultTable.getEntry("tv");
  public NetworkTableEntry tp = defaultTable.getEntry("pipeline");
  public NetworkTableEntry ledMode = defaultTable.getEntry("ledMode");
  //public NetworkTableEntry stream = defaultTable.getEntry("stream");

  public NetworkTable collectTable = NetworkTableInstance.getDefault().getTable("limelight-collect");
  public NetworkTableEntry c_tx = collectTable.getEntry("tx");
  public NetworkTableEntry c_ty = collectTable.getEntry("ty");
  public NetworkTableEntry c_tv = collectTable.getEntry("tv");
  public NetworkTableEntry c_tp = collectTable.getEntry("pipeline");
  public NetworkTableEntry c_ledMode = collectTable.getEntry("ledMode");

  public double p;
  public double x;
  public double y;
  public double v;
  public double led;

  public double p_collect;
  public double x_collect;
  public double y_collect;
  public double v_collect;
  public double led_collect;

  /** Creates a new Limelight. */
  public Limelight() {

    setPipeline(0);
    
  }

  @Override
  public void periodic() {

    p = tp.getDouble(-1.0);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(-1.0);
    led = ledMode.getDouble(-1.0);
    // This method will be called once per scheduler run
  }

  public void setPipeline(int pipelineId) {
    defaultTable.getEntry("pipeline").setInteger(pipelineId);
    collectTable.getEntry("pipeline").setInteger(pipelineId);

    p = pipelineId;
  }

  public double getPipeline() {
    p = tp.getDouble(-1.0);
    return p;
  }

  public double getPipelineCollect() {
    p = c_tp.getDouble(-1.0);
    return p;
  }

  public double getX() {
    x = tx.getDouble(0.0);
    return x;
  }

  public double getXCollect() {
    x_collect = c_tx.getDouble(0.0);
    return x_collect;
  }

  public double getFovX() {
    return fovX;
  }

  public double getXProportional() {
    return getX() / (getFovX() / 2);
  }
  
  public double getXProportionalCollect() {
    return getXCollect() / (getFovX() / 2);
  }

  public double getY() {
    y = ty.getDouble(0.0);
    return y;
  }

  public double getYCollect() {
    y = c_ty.getDouble(0.0);
    return y;
  }

  public double getV(){
    v = tv.getDouble(0.0); // should this be a boolean?
    return v;
  }

  public double getVCollect(){
    v = c_tv.getDouble(0.0); // should this be a boolean?
    return v;
  }

  public void setLedMode(int mode) {
    this.ledMode.setNumber(mode);
   // this.c_ledMode.setNumber(mode);
  }

  public void setLedModeCollect(int mode) {
    this.c_ledMode.setNumber(mode);
  }

  public double getDistanceToTarget(int target) {
    double goalHeightInches = 0.0;

    switch(target) {
      case TargetType.CONE_HIGH:
        goalHeightInches = Constants.LimelightConstants.highConeTargetHeight;
        break;

      case TargetType.CONE_MID:
        goalHeightInches = Constants.LimelightConstants.midConeTargetHeight;
        break;

      default:
        goalHeightInches = Constants.LimelightConstants.aprilTagTargetHeight;
        break;
      
    }

    double angleToGoalDegrees = Constants.LimelightConstants.mountAngleDegrees + getY();
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    return (goalHeightInches - Constants.LimelightConstants.LensHeightInches) / Math.tan(angleToGoalRadians);

  }
}

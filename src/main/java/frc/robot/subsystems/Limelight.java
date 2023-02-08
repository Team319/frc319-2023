// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private static double fovX = 54.0;
  private static double fovY = 41.0;

  public NetworkTable defaultTable = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry tx = defaultTable.getEntry("tx");
  public NetworkTableEntry ty = defaultTable.getEntry("ty");
  public NetworkTableEntry tv = defaultTable.getEntry("tv");
  public NetworkTableEntry tp = defaultTable.getEntry("pipeline");
  public NetworkTableEntry ledMode = defaultTable.getEntry("ledMode");
  //public NetworkTableEntry stream = defaultTable.getEntry("stream");

  public double p;
  public double x;
  public double y;
  public double v;
  public double led;

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
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getFovX() {
    return fovX;
  }

  public double getXProportional() {
    return getX() / (getFovX() / 2);
  }

  public void setLedMode(int mode) {
    this.ledMode.setNumber(mode);
  }
}

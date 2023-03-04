// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class LEDColor extends CommandBase {

  private int rValue = 0;
  private int gValue = 0;
  private int bValue = 0;

  /** Creates a new LEDColor. */
  public LEDColor(int rValue, int gValue, int bValue) {
    this.rValue = rValue;
    this.gValue = gValue;
    this.bValue = bValue;
    addRequirements(Robot.leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.leds.colorTest(rValue, gValue, bValue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

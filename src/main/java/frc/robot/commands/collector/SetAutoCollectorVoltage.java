// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SetAutoCollectorVoltage extends CommandBase {
  /** Creates a new SetAutoCollectorVoltage. */
  private double voltage = 0.0;
  private int i;

  public SetAutoCollectorVoltage(double voltage) {
    this.voltage = voltage;
    addRequirements(Robot.collector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    Robot.collector.setCollectorCurrentLimit(30);
    Robot.collector.setCollectorPO(voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.collector.setCollectorCurrentLimit(15);
    Robot.collector.setCollectorPO(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    i++;
    if (i > 25) {
      return true;
    }
    
    else {
      return false;
    }
  }
}

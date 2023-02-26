// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorMoveALittle extends CommandBase {
  private double movement = 0.0;
  private double collectVal = 0.0;
  private double initialPosition = 0.0;
  /** Creates a new ElevatorMoveALittle. */
  public ElevatorMoveALittle(double movement, double collectVal) {
    this.movement = movement;
    this.collectVal = collectVal;
    addRequirements(Robot.elevator, Robot.collector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initialPosition = Robot.elevator.getCurrentPosition();
    Robot.collector.setCollectorCurrentLimit(Constants.CollectorConstants.Currents.currentMax);
    Robot.collector.setCollectorPO(collectVal);
    Robot.elevator.setPosition(initialPosition + movement);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.collector.setCollectorCurrentLimit(Constants.CollectorConstants.Currents.currentThreshold);
    Robot.collector.setCollectorPO(0.15);
    Robot.elevator.setPosition(initialPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

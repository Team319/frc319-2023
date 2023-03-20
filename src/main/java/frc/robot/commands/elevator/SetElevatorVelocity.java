// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SetElevatorVelocity extends CommandBase {

  private double velocity = 0.0;
  /** Creates a new SetElevatorVelocity. */
  public SetElevatorVelocity(double velocity) {
    this.velocity = velocity;
    addRequirements(Robot.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Robot.elevator.setElevatorVelocity(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Robot.elevator.setElevatorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elbow;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.HelperFunctions;

public class ElbowSetSmartPosition extends CommandBase {
  private double position = 0.0;
  /** Creates a new ElbowSetSmartPosition. */
  public ElbowSetSmartPosition(double position) {
    this.position = position;
    addRequirements(Robot.elbow);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.elbow.setSmartMotionPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return HelperFunctions.deadband(Robot.elbow.getCurrentPosition() - position, Constants.ElbowConstants.SetPoints.deadband) == 0.0;
  }
}

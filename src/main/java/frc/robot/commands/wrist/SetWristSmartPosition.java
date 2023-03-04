// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SetWristSmartPosition extends CommandBase {
  private double position = 0.0;
  /** Creates a new WristSetSmartPosition. */
  public SetWristSmartPosition(double position) {
    this.position = position;
    addRequirements(Robot.wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.wrist.setSmartMotionPosition(position);
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
    //return HelperFunctions.deadband(Robot.wrist.getCurrentPosition() - position, Constants.WristConstants.SetPoints.deadband) == 0.0;
    return false;
  }
}

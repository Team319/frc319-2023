// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class GetDistance extends CommandBase {
  /** Creates a new GetDistance. */
  public GetDistance() {
    addRequirements(Robot.limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.limelight.setLedMode(Constants.LimelightConstants.LED_ON);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (Robot.limelight.getDistanceToTarget(Constants.TargetType.CONE_HIGH))
    //Robot.limelight.getDistanceToTarget(Constants.TargetType.CONE_HIGH);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.limelight.setLedMode(Constants.LimelightConstants.LED_OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.limelight.getDistanceToTarget(Constants.TargetType.CONE_HIGH) <= 10;
  }
}

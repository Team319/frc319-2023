// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants.DriveMode;

public class SetDriveMode extends CommandBase {
  /** Creates a new SetDriveMode. */
  public DriveMode drivemode = DriveMode.Normal;
  public SetDriveMode(DriveMode drivemode) {
    this.drivemode = drivemode;
    addRequirements(Robot.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.setDriveMode(drivemode);
    if (drivemode == DriveMode.Normal || drivemode == DriveMode.Scoring) {
      Robot.limelight.setLedMode(1);
    }
    else {
      Robot.limelight.setLedMode(3);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

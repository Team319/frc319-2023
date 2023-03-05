// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.BobDriveHelper;
import frc.robot.utils.DriveSignal;

public class DriveToPitch extends CommandBase {
  
  double pitchLimit = 0;
  double moveValue = 0.3;

  BobDriveHelper helper = new BobDriveHelper();
  DriveSignal driveSignal = helper.cheesyDrive(moveValue, 0, false, false);
  
  /** Creates a new DriveToPitch. */
  public DriveToPitch(double pitch) {
    addRequirements(Robot.drivetrain);
    pitchLimit = pitch;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSignal = helper.cheesyDrive(0, 0, false, false);
    Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.drivetrain.getPitch() >= pitchLimit;
  }
}

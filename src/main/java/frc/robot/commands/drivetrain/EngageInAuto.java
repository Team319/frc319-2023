// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utils.BobDriveHelper;
import frc.robot.utils.DriveSignal;
import frc.robot.utils.HelperFunctions;

public class EngageInAuto extends CommandBase {

  double pitchLimit = 0;
  double moveValue = 0.008;//0.0105
  double levelCount = 0;
  double deadband = 3.0;

  BobDriveHelper helper = new BobDriveHelper();
  DriveSignal driveSignal = helper.cheesyDrive(moveValue, 0, false, false);
  
  /** Creates a new EngageInAuto. */
  public EngageInAuto() {
    addRequirements(Robot.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (HelperFunctions.deadband(Robot.drivetrain.getPitch(), deadband) != 0) {
      levelCount = 0;
      driveSignal = helper.cheesyDrive(moveValue*Robot.drivetrain.getPitch(), 0, false, false);
      Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
      // if (Robot.drivetrain.getPitch() > 5) {
      //   driveSignal = helper.cheesyDrive(moveValue, 0, false, false);
      //   Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
      // }
      // if (Robot.drivetrain.getPitch() < -8) {
      //   driveSignal = helper.cheesyDrive(moveValue*-1, 0, false, false);
      //   Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
      
      // } 
      // else {
      //   driveSignal = helper.cheesyDrive(0, 0, false, false);
      //   Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
      // }
    // } else {
    //     levelCount++;
    //     driveSignal = helper.cheesyDrive(0, 0, false, false);
    //     Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return levelCount >= 1;
  }
}

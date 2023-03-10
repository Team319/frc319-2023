// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveToPitch;
import frc.robot.commands.drivetrain.EngageInAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class AutoDriveForwardAndEngage extends SequentialCommandGroup {

  // Creating AutoDriveForwardAndEngage command
  public AutoDriveForwardAndEngage() {

    // Adding required commands
    addCommands(new DriveToPitch(13), new EngageInAuto());
  }
}

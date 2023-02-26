// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.collector.SetCollectorVelocity;
import frc.robot.commands.drivetrain.SetDriveMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitGamePiece extends SequentialCommandGroup {
  /** Creates a new SpitGamePiece. */
  public SpitGamePiece(double velocity) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetCollectorVelocity(velocity)
      //new SetDriveMode(DriveMode.Normal)
    );
  }
}
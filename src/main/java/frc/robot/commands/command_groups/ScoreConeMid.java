// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.drivetrain.SetDriveMode;
import frc.robot.commands.elbow.ElbowGoToPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.wrist.WristGoToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeMid extends SequentialCommandGroup {
  /** Creates a new ScoreConeMid. */
  public ScoreConeMid() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorPosition(Constants.ElevatorConstants.SetPoints.scoreMiddleCone), 
      new WaitCommand(0.5),
      new ElbowGoToPosition(Constants.ElbowConstants.SetPoints.scoreMiddleCone),
      new WristGoToPosition(Constants.WristConstants.SetPoints.scoreMiddleCone)
      //new SetDriveMode(DriveMode.Scoring)
    );
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elbow.SetElbowPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.limelight.SwitchingPipelineTest;
import frc.robot.commands.wrist.SetWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeHigh extends SequentialCommandGroup {
  /** Creates a new ScoreConeMid. */
  public ScoreConeHigh() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands (
      new SetElevatorPosition(Constants.ElevatorConstants.SetPoints.scoreHighCone), 
      new SetElbowPosition(Constants.ElbowConstants.SetPoints.scoreHighCone),
      new SetWristPosition(Constants.WristConstants.SetPoints.scoreHighCone),
      new SwitchingPipelineTest(Constants.LimelightConstants.Modes.LIMELIGHT_TOP)
      //new SetDriveMode(DriveMode.Scoring)
    );
  }
}

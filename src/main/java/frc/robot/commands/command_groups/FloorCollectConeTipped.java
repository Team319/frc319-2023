// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.collector.SetCollectorVoltage;
import frc.robot.commands.elbow.ElbowGoToPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.wrist.WristGoToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FloorCollectConeTipped extends SequentialCommandGroup {
  /** Creates a new FloorCollectConeTipped. */
  public FloorCollectConeTipped() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorPosition(19), 
      new WristGoToPosition(-63),
      new ElbowGoToPosition(-0.148682)
    );
  }
}

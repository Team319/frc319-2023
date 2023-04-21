// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.CollectorState;
import frc.robot.commands.collector.SetCollectorVoltage;
import frc.robot.commands.elbow.SetElbowPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.wrist.SetWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectConeFromLoadStation extends SequentialCommandGroup {
  /** Creates a new CollectConeFromLoadStation. */
  public CollectConeFromLoadStation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorPosition(Constants.ElevatorConstants.SetPoints.collectConeFromLoadStation),
      new SetWristPosition(Constants.WristConstants.SetPoints.collectFromLoadStation),
      new SetElbowPosition(Constants.ElbowConstants.SetPoints.collectFromLoadStation),
      new SetCollectorVoltage(Constants.CollectorConstants.Currents.collectorVoltage, 15)
    );

    Robot.collectorState = CollectorState.HOLDING_CONE;
    
  }
}

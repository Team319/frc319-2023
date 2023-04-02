// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.command_groups;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.elbow.SetElbowPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.leds.LEDColor;
import frc.robot.commands.wrist.SetWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoHomeFast extends SequentialCommandGroup {
  /** Creates a new GoHome. */
 public GoHomeFast()  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

      addCommands(
        Commands.parallel(
          //new MoveWristAndElbow(Constants.WristConstants.SetPoints.home, Constants.ElbowConstants.SetPoints.home),
          new SetElevatorPosition(Constants.ElevatorConstants.SetPoints.home),
          new SetWristPosition(Constants.WristConstants.SetPoints.home),
          new SetElbowPosition(Constants.ElbowConstants.SetPoints.home)
        )
      );
  }
}

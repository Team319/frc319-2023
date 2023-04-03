// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.command_groups.FloorCollect;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestBlueLeft extends SequentialCommandGroup {

  Trajectory blueLeftCharge1 = Robot.drivetrain.loadTrajectoryFromFile("blueleftcharge1");
  Trajectory blueLeftCharge2 = Robot.drivetrain.loadTrajectoryFromFile("blueleftcharge2");
  
  /** Creates a new TestBlueLeft. */
  public TestBlueLeft() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        () -> {
        }),
        Commands.parallel(
        new InstantCommand(()->Robot.drivetrain.resetOdometry(blueLeftCharge1.getInitialPose())),
        Robot.drivetrain.createCommandForTrajectory(blueLeftCharge1, false)),

        Commands.parallel(Robot.drivetrain.createCommandForTrajectory(blueLeftCharge2, false))
        );
  }
}

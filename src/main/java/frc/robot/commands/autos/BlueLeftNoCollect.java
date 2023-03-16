// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.command_groups.AutoScoreHigh;
import frc.robot.commands.command_groups.FloorCollect;
import frc.robot.commands.command_groups.GoHome;
import frc.robot.commands.command_groups.PreScorePosition;
import frc.robot.commands.command_groups.ScoreConeHigh;
import frc.robot.commands.command_groups.SpitGamePiece;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueLeftNoCollect extends SequentialCommandGroup {

  Trajectory blueLeftNoCollect1 = Robot.drivetrain.loadTrajectoryFromFile("blueleftnocollect1");


  /** Creates a new RedRight. */
  public BlueLeftNoCollect() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        () -> {

      }),
      new AutoScoreHigh(),
      new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                new SpitGamePiece(-1)),
      new PreScorePosition(),

      Commands.parallel(
        new InstantCommand(()->Robot.drivetrain.resetOdometry(blueLeftNoCollect1.getInitialPose())),
        Robot.drivetrain.createCommandForTrajectory(blueLeftNoCollect1, false)),
        
      new ParallelDeadlineGroup(new AutoDriveForwardAndEngage())
      
      
    );
  }
}
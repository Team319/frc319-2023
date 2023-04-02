// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.collector.SetAutoCollectorVoltage;
import frc.robot.commands.command_groups.AutoScoreConeHigh;
import frc.robot.commands.command_groups.AutoScoreCubeHigh;
import frc.robot.commands.command_groups.AutoScoreHigh;
import frc.robot.commands.command_groups.FloorCollect;
import frc.robot.commands.command_groups.GoHome;
import frc.robot.commands.command_groups.PreScorePosition;
import frc.robot.commands.command_groups.SpitGamePiece;
import frc.robot.commands.drivetrain.DriveToPitch;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedRight2AndCharge extends SequentialCommandGroup {

  Trajectory redRight2AndCharge1 = Robot.drivetrain.loadTrajectoryFromFile("redright2andcharge1");
  Trajectory redRight2AndCharge2 = Robot.drivetrain.loadTrajectoryFromFile("redright2andcharge2");
  Trajectory redRight2AndCharge3 = Robot.drivetrain.loadTrajectoryFromFile("redright2andcharge3");

  /** Creates a new RedRight2AndCharge. */
  public RedRight2AndCharge() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
      () -> {
      }),
      new AutoScoreConeHigh(),
        new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                  new SpitGamePiece(-0.75)),
      Commands.parallel(
          new InstantCommand(()->Robot.drivetrain.resetOdometry(redRight2AndCharge1.getInitialPose())),
          Robot.drivetrain.createCommandForTrajectory(redRight2AndCharge1, false),
          Commands.race(new WaitCommand(3.5), new FloorCollect())
          ),
          

          Commands.parallel(
          Robot.drivetrain.createCommandForTrajectory(redRight2AndCharge2, false),
          Commands.sequence(new GoHome(), new AutoScoreCubeHigh())

        ),
       
        new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                  new SpitGamePiece(-1)),
        

        Commands.sequence(
          Commands.parallel(
            new GoHome(),
            Robot.drivetrain.createCommandForTrajectory(redRight2AndCharge3, false)
        ),
          new AutoDriveForwardAndEngage()
        )
      );
  }
}

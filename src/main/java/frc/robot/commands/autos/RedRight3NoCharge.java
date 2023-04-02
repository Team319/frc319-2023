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
import frc.robot.commands.command_groups.AutoFloorCollectConeTipped;
import frc.robot.commands.command_groups.AutoScoreCubeHigh;
import frc.robot.commands.command_groups.AutoScoreCubeMid;
import frc.robot.commands.command_groups.AutoScoreHigh;
import frc.robot.commands.command_groups.AutoScoreMid;
import frc.robot.commands.command_groups.FloorCollect;
import frc.robot.commands.command_groups.FloorCollectConeTipped;
import frc.robot.commands.command_groups.GoHome;
import frc.robot.commands.command_groups.PreScorePosition;
import frc.robot.commands.command_groups.SpitGamePiece;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedRight3NoCharge extends SequentialCommandGroup {

  Trajectory redRight3NoCharge1 = Robot.drivetrain.loadTrajectoryFromFile("redright2andcharge1");
  Trajectory redRight3NoCharge2 = Robot.drivetrain.loadTrajectoryFromFile("redright2andcharge2");
  Trajectory redRight3NoCharge3 = Robot.drivetrain.loadTrajectoryFromFile("redright3gamepiece3");
  Trajectory redRight3NoCharge4 = Robot.drivetrain.loadTrajectoryFromFile("redright3gamepiece4");
  
  /** Creates a new RedRight3NoCharge. */
  public RedRight3NoCharge() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
      () -> {
      }),

      new AutoScoreMid(),
        new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                  new SpitGamePiece(-1)),
      Commands.parallel(
          new InstantCommand(()->Robot.drivetrain.resetOdometry(redRight3NoCharge1.getInitialPose())),
          Robot.drivetrain.createCommandForTrajectory(redRight3NoCharge1, false),new FloorCollect()
          ),
          

          Commands.parallel(
          Robot.drivetrain.createCommandForTrajectory(redRight3NoCharge2, false),
          Commands.sequence(new GoHome(),new PreScorePosition(),new AutoScoreCubeMid())

        ),
       
        new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                  new SpitGamePiece(-1)),
        


        //new SetAutoCollectorVoltage(-1),
        Commands.sequence(
          Commands.parallel(
            Robot.drivetrain.createCommandForTrajectory(redRight3NoCharge3, false),new AutoFloorCollectConeTipped()
        )),

        Commands.sequence(
          Commands.parallel(
            Robot.drivetrain.createCommandForTrajectory(redRight3NoCharge4, false), new GoHome()
          )
        )
    );
  }
}

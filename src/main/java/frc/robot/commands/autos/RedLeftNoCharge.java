// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.collector.SetAutoCollectorVoltage;
import frc.robot.commands.collector.SetCollectorVoltage;
import frc.robot.commands.command_groups.AutoScoreConeHigh;
import frc.robot.commands.command_groups.AutoScoreCubeHigh;
import frc.robot.commands.command_groups.AutoScoreHigh;
import frc.robot.commands.command_groups.FloorCollect;
import frc.robot.commands.command_groups.FloorCollectConeStanding;
import frc.robot.commands.command_groups.GoHome;
import frc.robot.commands.command_groups.PreScorePosition;
import frc.robot.commands.command_groups.ScoreConeHigh;
import frc.robot.commands.command_groups.SpitGamePiece;
import frc.robot.commands.elbow.SetElbowPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.wrist.SetWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedLeftNoCharge extends SequentialCommandGroup {
  /** Creates a new RedRight. */

  Trajectory redLeftNoCharge1 = Robot.drivetrain.loadTrajectoryFromFile("redleftnocharge1");
  Trajectory redLeftNoCharge2 = Robot.drivetrain.loadTrajectoryFromFile("redleftnocharge2");
  Trajectory redLeftNoCharge3 = Robot.drivetrain.loadTrajectoryFromFile("redleftnocharge3");


  public RedLeftNoCharge() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
    addCommands(
      new InstantCommand(
        () -> {}
      ),
        
      new AutoScoreConeHigh(),
      new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                new SpitGamePiece(-1)),
      Commands.parallel(
        new InstantCommand(()->Robot.drivetrain.resetOdometry(redLeftNoCharge1.getInitialPose())),
        Robot.drivetrain.createCommandForTrajectory(redLeftNoCharge1, false),
        Commands.race(new WaitCommand(3.5), new FloorCollect())),
        
      Commands.sequence(
        Commands.parallel(Robot.drivetrain.createCommandForTrajectory(redLeftNoCharge2, false),
                        new GoHome()),
                        new AutoScoreCubeHigh(),
                        new SetAutoCollectorVoltage(-1)),
      new GoHome(),
      Commands.parallel(Robot.drivetrain.createCommandForTrajectory(redLeftNoCharge3, false),
      Commands.race(new WaitCommand(3.5), new FloorCollectConeStanding())
      )

    );
  }
}

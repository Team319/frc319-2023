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
import frc.robot.commands.elbow.SetElbowPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.wrist.SetWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedLeft extends SequentialCommandGroup {

  Trajectory redLeftCharge1 = Robot.drivetrain.loadTrajectoryFromFile("redleftcharge1");
  Trajectory redLeftCharge2 = Robot.drivetrain.loadTrajectoryFromFile("redleftcharge2");

  /** Creates a new RedRight. */
  public RedLeft() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        () -> {
        }),
      /*new AutoScoreHigh(),
  
      new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                new SpitGamePiece(-1)),*/

      Commands.parallel(
        new InstantCommand(()->Robot.drivetrain.resetOdometry(redLeftCharge1.getInitialPose())),
        Robot.drivetrain.createCommandForTrajectory(redLeftCharge1, false)
        
        /*Commands.sequence(
          new PreScorePosition(),
          Commands.parallel(
              new SetElevatorPosition(Constants.ElevatorConstants.SetPoints.collectFloor), 
              new SetElbowPosition(Constants.ElbowConstants.SetPoints.collectFloor),
              new SetWristPosition(Constants.WristConstants.SetPoints.collectFloor)
            )
      
          )*/
      ),
      Commands.parallel(Robot.drivetrain.createCommandForTrajectory(redLeftCharge2, false) 
                              /*Commands.sequence( 
                                Commands.race(new WaitCommand(1.25), new FloorCollect())
                              )*/

      )

      //new ParallelCommandGroup(new AutoDriveForwardAndEngage(),new GoHome())
    );
  }
}

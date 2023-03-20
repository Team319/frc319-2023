// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueLeft2AndCharge extends SequentialCommandGroup {

  Trajectory blueLeft2AndCharge1 = Robot.drivetrain.loadTrajectoryFromFile("blueleft2andcharge1");
  Trajectory blueLeft2AndCharge2 = Robot.drivetrain.loadTrajectoryFromFile("blueleft2andcharge2");
  Trajectory blueLeft2AndCharge3 = Robot.drivetrain.loadTrajectoryFromFile("blueleft2andcharge3");

  /** Creates a new BlueLeft2AndCharge. */
  public BlueLeft2AndCharge() {
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
          new InstantCommand(()->Robot.drivetrain.resetOdometry(blueLeft2AndCharge1.getInitialPose())),
          Robot.drivetrain.createCommandForTrajectory(blueLeft2AndCharge1, false)
          /*Commands.sequence(
            new PreScorePosition(),
            Commands.parallel(
              new SetElevatorPosition(Constants.ElevatorConstants.SetPoints.collectFloor), 
              new SetElbowPosition(Constants.ElbowConstants.SetPoints.collectFloor),
              new SetWristPosition(Constants.WristConstants.SetPoints.collectFloor)
            )
          )*/
          ),

          Commands.parallel(
          Robot.drivetrain.createCommandForTrajectory(blueLeft2AndCharge2, false)//,
          /*Commands.sequence( Commands.race( new WaitCommand(2.5) , new FloorCollect()),
                            new PreScorePosition(), 
                            new AutoScoreCubeHigh()                            
          )*/  

        ),

        //new SetAutoCollectorVoltage(-1),

        Commands.parallel(
          Robot.drivetrain.createCommandForTrajectory(blueLeft2AndCharge3, false) 

        )
      
      );
  }
}

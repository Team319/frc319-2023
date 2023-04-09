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
import frc.robot.commands.command_groups.AutoScoreConeHigh;
import frc.robot.commands.command_groups.AutoScoreCubeMid;
import frc.robot.commands.command_groups.AutoScoreHighCubeFast;
import frc.robot.commands.command_groups.FloorCollect;
import frc.robot.commands.command_groups.FloorCollectConeStanding;
import frc.robot.commands.command_groups.GoHome;
import frc.robot.commands.command_groups.SpitGamePiece;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueLeftScore3Cube2 extends SequentialCommandGroup {

  Trajectory blueLeft3Engage1 = Robot.drivetrain.loadTrajectoryFromFile("blueleft2andcharge1");
  Trajectory blueLeft3Engage2 = Robot.drivetrain.loadTrajectoryFromFile("blueleft2andcharge2");
  Trajectory blueLeft3NoCharge3 = Robot.drivetrain.loadTrajectoryFromFile("blueleft3gamepiece3");
  Trajectory blueLeft3NoCharge4 = Robot.drivetrain.loadTrajectoryFromFile("blueleft3lastcubemid4");
  
  /** Creates a new BlueLeftScore3Cube2. */
  public BlueLeftScore3Cube2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
      () -> {
      }),

      new AutoScoreConeHigh(),
      new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                  new SpitGamePiece(-1)),

      Commands.parallel(
        Commands.race(
          new WaitCommand(3.5), 
          new FloorCollect()), 
          Commands.sequence(
            new WaitCommand(0.125)),
            new InstantCommand(()->Robot.drivetrain.resetOdometry(blueLeft3Engage1.getInitialPose())),
            Robot.drivetrain.createCommandForTrajectory(blueLeft3Engage1, false)
          ),
        
        Commands.parallel(
          Robot.drivetrain.createCommandForTrajectory(blueLeft3Engage2, false),
          Commands.sequence(
            new GoHome(),
            new AutoScoreHighCubeFast())

        ),
        new WaitCommand(0.25),
        new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                  new SpitGamePiece(-1)),
        


        //new SetAutoCollectorVoltage(-1),
        Commands.sequence(
          Commands.parallel(
            Robot.drivetrain.createCommandForTrajectory(blueLeft3NoCharge3, false),
            new FloorCollect()
            //Commands.race(new WaitCommand(3.25), new FloorCollectConeStanding()) 
        )),

          Commands.parallel(
            Robot.drivetrain.createCommandForTrajectory(blueLeft3NoCharge4, false), 
            Commands.sequence(
              new GoHome(),
              new WaitCommand(0.5),
              new AutoScoreCubeMid()
              //new PreScorePosition())
            )),
          new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                      new SpitGamePiece(-1)),
          new GoHome()

            //,
          // Commands.parallel(
          //   new PreScorePosition(),
          //   new ParallelDeadlineGroup(new WaitCommand(0.25), 
          //                         new SpitGamePiece(-1))
          // )
    );
  }
}

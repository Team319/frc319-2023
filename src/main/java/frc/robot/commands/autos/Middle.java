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
public class Middle extends SequentialCommandGroup {

  //Trajectory middlePath1 = Robot.drivetrain.loadTrajectoryFromFile("bluemiddle1");
  //Trajectory middlePath2 = Robot.drivetrain.loadTrajectoryFromFile("bluemiddle2");



  /** Creates a new RedRight. */
  public Middle() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        () -> {
        }),
      new AutoScoreHigh(),
  
      new ParallelDeadlineGroup(new WaitCommand(0.25), 
                                new SpitGamePiece(-0.75)),

      Commands.sequence(
                        new GoHome(),
                        new WaitCommand(0.5), // Let the elevator settle before we start moving untill we see some pitch
                        new AutoDriveAndEngage() // Dumb mobility over the C.S. and engage again
                       )

                       // EKM - we can test AutoDriveAndEngage in the pits


                       // Backup solution using paths... more risky
      /*Commands.sequence(new GoHome(), 
                        new InstantCommand(()->Robot.drivetrain.resetOdometry(middlePath1.getInitialPose())),
                        Robot.drivetrain.createCommandForTrajectory(middlePath1, false),
                        Robot.drivetrain.createCommandForTrajectory(middlePath2, false),
                        new AutoDriveForwardAndEngage()) // which way is forward? */
                        
      
     
    );
    
  }
}

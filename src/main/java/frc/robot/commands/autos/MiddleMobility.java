// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.command_groups.AutoScoreHigh;
import frc.robot.commands.command_groups.GoHome;
import frc.robot.commands.command_groups.SpitGamePiece;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleMobility extends SequentialCommandGroup {
  /** Creates a new MiddleMobility. */
  public MiddleMobility() {
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
                        new AutoMobilityAndEngage() // Dumb mobility over the C.S. and engage again
                       )
    );
  }
}

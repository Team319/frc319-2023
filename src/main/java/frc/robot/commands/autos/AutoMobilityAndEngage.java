// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveToPitch;
import frc.robot.commands.drivetrain.EngageInAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMobilityAndEngage extends SequentialCommandGroup {
  /** Creates a new AutoMobilityAndEngage. */
  public AutoMobilityAndEngage() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      Commands.deadline(new WaitCommand(10), // If 10 seconds pass, this should end, please don't cross centerline
      
        Commands.sequence(
          // TODO - verify move direction(s) is + = forward , and - = backward ( this should be fine, check in pits that it seems logical)
          new DriveToPitch(13, 0.5), // Drive until we see we are pitched
          new DriveToPitch(0, 0.3), // Drive until we see we are leveled on the platform
          new DriveToPitch(-13, 0.3), // Drive until we see we are pitched again
          new DriveToPitch(0, 0.5), // Drive until we see we are leveled on the floor
          // if we see zero, the whole robot frame should be off the station
          new WaitCommand(0.5), // TODO : Update or remove. Do we need a " drive for x seconds command? "
          new DriveToPitch(-13, -0.5), // Drive until we see we are pitched
          new EngageInAuto() // In theory this will drive the correct direction depending on the pitch
          ) // Last thing that we do, to level out
        )
        
      );
  }
}

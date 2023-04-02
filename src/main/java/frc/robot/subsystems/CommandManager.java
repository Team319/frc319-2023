package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.command_groups.GoHome;
import frc.robot.commands.command_groups.GoHomeFast;

public class CommandManager extends CommandBase {
    
    public CommandManager() {
      addRequirements(Robot.elevator);
    };


    public Command goHomeSmart() {
        if (Robot.elevator.getCurrentPosition() > Constants.ElevatorConstants.SetPoints.preScore) {
          return new GoHomeFast();
        }
        return new GoHome();
      }

}

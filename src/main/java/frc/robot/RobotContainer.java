// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.drivetrain.SetDriveMode;
import frc.robot.commands.wrist.SetWristPosition;
import frc.robot.commands.collector.SetCollectorVoltage;
import frc.robot.commands.command_groups.CollectConeFromLoadStation;
import frc.robot.commands.command_groups.CollectCubeFromLoadStation;
import frc.robot.commands.command_groups.FloorCollect;
import frc.robot.commands.command_groups.FloorCollectConeStanding;
import frc.robot.commands.command_groups.FloorCollectConeTipped;
import frc.robot.commands.command_groups.GoHome;
import frc.robot.commands.command_groups.PreScorePosition;
import frc.robot.commands.command_groups.ScoreConeHigh;
import frc.robot.commands.command_groups.ScoreConeMid;
import frc.robot.commands.command_groups.SpitGamePiece;
import frc.robot.commands.elevator.ElevatorMoveALittle;
import frc.robot.commands.leds.LEDColor;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    /* Driver Controllers */
    //m_driverController.a().onTrue(new WristGoToPosition(Constants.WristConstants.SetPoints.scoreHighCone));
    //m_driverController.b().onTrue(new WristGoToPosition(Constants.WristConstants.SetPoints.bottom));
    //m_driverController.x().whileTrue(new SwitchingPipelineTest(Constants.LimelightConstants.Modes.LIMELIGHT_TOP));
    //m_driverController.y().whileTrue(new SetElevatorPosition(19));

    /*m_driverController.povUp().whileTrue(new SetElevatorVoltage(0.4));
    m_driverController.povLeft().whileTrue(new SetElevatorVoltage(0.0));
    m_driverController.povDown().whileTrue(new SetElevatorVoltage(-0.4));*/

    //m_driverController.leftBumper().whileTrue(new LEDColor(0, 0, 255));
    m_driverController.leftBumper().whileTrue( new SetDriveMode(DriveMode.Limelight) );
    m_driverController.leftBumper().onFalse(new SetDriveMode(DriveMode.Normal));

    m_driverController.leftTrigger().whileTrue(new SetDriveMode(DriveMode.Scoring));
    m_driverController.leftTrigger().onFalse(new SetDriveMode(DriveMode.Normal));
    m_driverController.rightTrigger().whileTrue(new SetCollectorVoltage(-Constants.CollectorConstants.Currents.collectorVoltage));
    m_driverController.rightBumper().whileTrue(new SetCollectorVoltage(Constants.CollectorConstants.Currents.collectorVoltage));
    //m_driverController.leftBumper().whileTrue(new SetCollectorVoltage(-Constants.CollectorConstants.Currents.collectorVoltage));

    /* Operator Controllers */
    m_operatorController.a().onTrue(new FloorCollect());
    m_operatorController.x().onTrue(new FloorCollectConeStanding());
    m_operatorController.y().onTrue(new FloorCollectConeTipped());
    m_operatorController.b().onTrue(new GoHome());

    m_operatorController.povUp().onTrue(new PreScorePosition());
    m_operatorController.povRight().onTrue(new ScoreConeMid());
    m_operatorController.povLeft().onTrue(new ScoreConeHigh());
    m_operatorController.povDown().onTrue(new SetWristPosition(Constants.WristConstants.SetPoints.home));

    m_operatorController.leftBumper().onTrue(new CollectConeFromLoadStation());
    m_operatorController.leftTrigger().whileTrue(new ElevatorMoveALittle(-4, Constants.CollectorConstants.Currents.collectorVoltage));
    m_operatorController.rightBumper().onTrue(new CollectCubeFromLoadStation());
    m_operatorController.rightTrigger().whileTrue(new ElevatorMoveALittle(4, Constants.CollectorConstants.Currents.collectorVoltage));

    //m_operatorController.leftBumper().whileTrue(new LEDColor(255, 0, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
   // // An example command will be run in autonomous
   // return new Command();
  //}

  public Pair<Double, Double> getLeftStick() {
    double leftX = m_driverController.getLeftX();
    double leftY = -1.0 * m_driverController.getLeftY();
    return new Pair<>(leftX, leftY);
  }

  public Pair<Double, Double> getRightStick() {
    double rightX = m_driverController.getRightX();
    double rightY = -1.0 * m_driverController.getRightY();
    return new Pair<>(rightX, rightY);
  }
}

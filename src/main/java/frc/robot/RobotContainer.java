// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivetrain.ResetOdometryAndHeading;
import frc.robot.commands.drivetrain.SetDriveMode;
import frc.robot.commands.limelight.SwitchingPipelineTest;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.autos.TestPath;
import frc.robot.commands.autos.elevator.SetElevatorPosition;
import frc.robot.commands.autos.elevator.SetElevatorVoltage;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.a().whileTrue(new SwitchingPipelineTest(Constants.LimelightConstants.Modes.APRIL_TAG_MODE));
    m_driverController.b().whileTrue(new SwitchingPipelineTest(Constants.LimelightConstants.Modes.LIMELIGHT_BOTTOM));
    m_driverController.x().whileTrue(new SwitchingPipelineTest(Constants.LimelightConstants.Modes.LIMELIGHT_TOP));
    //m_driverController.y().whileTrue(null);

    m_driverController.povUp().whileTrue(new SetElevatorPosition(70.0));
    m_driverController.povLeft().whileTrue(new SetElevatorPosition(35.0));
    m_driverController.povDown().whileTrue(new SetElevatorPosition(0.25));

    m_driverController.leftTrigger().whileTrue(new SetDriveMode());
    //m_driverController.rightTrigger().whileTrue(null);
    //m_driverController.leftBumper().whileTrue(null);
    //m_driverController.rightBumper().whileTrue(null);
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

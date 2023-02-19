// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.BobDrive;
import frc.robot.commands.autos.AutoDriveForwardAndEngage;
import frc.robot.commands.autos.TestPath;
import frc.robot.commands.drivetrain.DriveToPitch;
import frc.robot.commands.drivetrain.EngageInAuto;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  
  private Command m_autonomousCommand;;

  private Command m_teleopCommand = new BobDrive();
  public static Drivetrain drivetrain = new Drivetrain();
  public static Limelight limelight = new Limelight();
  public static Elevator elevator = new Elevator();
  public static Wrist wrist = new Wrist();
  public static Elbow elbow = new Elbow();
  public static Collector collector = new Collector();

  private RobotContainer m_robotContainer;
  public static Trajectory autoTrajectory = new Trajectory();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
    Robot.drivetrain.setDefaultCommand(new BobDrive());
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    String trajectoryJSON = "paths//blueleft.wpilib.json";
 
    try {
      Path testPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      autoTrajectory = TrajectoryUtil.fromPathweaverJson(testPath);
    }

    catch (IOException ex) {
      DriverStation.reportError("Unable to open Trajectory", ex.getStackTrace());
    } 

    drivetrain.zeroOdometry();
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    //SmartDashboard.putNumber("Left Distance", drivetrain.getLeftLeadDriveDistanceMeters());
    //SmartDashboard.putNumber("Right Distance", drivetrain.getRightLeadDriveDistanceMeters());
    //SmartDashboard.putNumber("Left Ticks", drivetrain.getLeftLeadDriveDistanceTicks());
    //SmartDashboard.putNumber("Right Ticks", drivetrain.getRightLeadDriveDistanceTicks());

    //SmartDashboard.putNumber("Pose X", drivetrain.getPose().getX());
    //SmartDashboard.putNumber("Pose Y", drivetrain.getPose().getY());
    //SmartDashboard.putNumber("Fused Heading", drivetrain.getHeadingDegrees());
    
    //SmartDashboard.putNumber("Left Wheel Speed", drivetrain.getLeftMotorSpeed());
    //SmartDashboard.putNumber("Right Wheel Speed", drivetrain.getRightMotorSpeed());
    //SmartDashboard.putNumber("Left Accel", drivetrain.pigeon.getAcc);

    //SmartDashboard.putNumber("Pitch", drivetrain.getPitch());
    //SmartDashboard.putNumber("Delta Pitch", drivetrain.getDeltaPitch());

    /*
     *   
     *   SmartDashboard.putNumber("Pipeline ID", limelight.p);
    SmartDashboard.putNumber("Target X", limelight.x);
    SmartDashboard.putNumber("Target Y", limelight.y);
    SmartDashboard.putNumber("Target V", limelight.v);
    SmartDashboard.putNumber("LED Mode", limelight.led);
     */


    SmartDashboard.putNumber("Drive Mode", drivetrain.getDriveMode().ordinal());

    SmartDashboard.putNumber("Elevator Position", elevator.getCurrentPosition());
    SmartDashboard.putNumber("Elevator Velocity", elevator.getVelocity());
    SmartDashboard.putNumber("Elbow Position", elbow.getCurrentPosition());
    SmartDashboard.putNumber("Elbow Current", elbow.getElbowCurrent());
    SmartDashboard.putNumber("Elbow Velocity", elbow.getElbowMotorVelocity());
    SmartDashboard.putNumber("Wrist Position", wrist.getCurrentPosition());
    SmartDashboard.putNumber("Wrist Current", wrist.getWristCurrent());
    SmartDashboard.putNumber("Wrist Velocity", wrist.getWristMotorVelocity());
    SmartDashboard.putNumber("Collector Current", collector.getCollectorCurrent());

    drivetrain.setFollowers();

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void disabledPeriodic() {
    Robot.elevator.setPosition(Robot.elevator.getCurrentPosition());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    Robot.drivetrain.setNeutralMode(NeutralMode.Brake);

    //m_autonomousCommand = new TestPath(autoTrajectory);
    //drivetrain.zeroOdometry(); //remove
    // schedule the autonomous command (example)
    //m_autonomousCommand = new DriveToPitch(10);
    m_autonomousCommand = new EngageInAuto();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    drivetrain.zeroOdometry(); //remove later
    Robot.elbow.setPosition(Robot.elbow.getCurrentPosition());
    Robot.wrist.setPosition(Robot.wrist.getCurrentPosition());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_teleopCommand.schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.security.Key;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CollectorState;
import frc.robot.commands.BobDrive;
import frc.robot.commands.autos.BlueLeft2AndCharge;
import frc.robot.commands.autos.BlueLeft3AndEngage;
import frc.robot.commands.autos.BlueLeft3NoCharge2Cube;
import frc.robot.commands.autos.BlueLeftScore3Cube2;
import frc.robot.commands.autos.BlueRightNoCharge;
import frc.robot.commands.autos.Middle;
import frc.robot.commands.autos.MultiPath;
import frc.robot.commands.autos.RedLeftNoCharge;
import frc.robot.commands.autos.RedRight2AndCharge;
import frc.robot.commands.autos.RedRight3AndEngage;
import frc.robot.commands.autos.RedRight3NoCharge;
import frc.robot.commands.autos.RedRight3NoCharge2Cube;
import frc.robot.commands.autos.RedRightScore3Cube2;
import frc.robot.commands.autos.ScoreConeHighAndWait;
import frc.robot.commands.autos.TestBlueLeft;
import frc.robot.commands.autos.TestFollowSplitPaths;
import frc.robot.commands.autos.TestPath;
import frc.robot.commands.command_groups.GoHome;
import frc.robot.commands.command_groups.GoHomeFast;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CommandManager;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.LEDS.Section;

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
  public static LEDS leds = new LEDS();
  
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  SendableChooser<Command> testAutoChooser = new SendableChooser<Command>();

  private RobotContainer m_robotContainer;

  public static Trajectory Trajectory1 = new Trajectory();
  public static Trajectory Trajectory2 = new Trajectory();
  public static Trajectory Trajectory3 = new Trajectory();

  //RedRightCharge
  public static Trajectory RedRightCharge1 = new Trajectory();
  public static Trajectory RedRightCharge2 = new Trajectory();
  public static Trajectory RedRightCharge3 = new Trajectory();
  
  public static Trajectory RedRight = new Trajectory();

  //RedRightNoCharge
  public static Trajectory RedRightNoCharge1 = new Trajectory();
  public static Trajectory RedRightNoCharge2 = new Trajectory();
  public static Trajectory RedRightNoCharge3 = new Trajectory();
  
  public static Trajectory RedRightNoChargeFirstPath = new Trajectory();
  public static Trajectory RedRightNoChargeSecondPath = new Trajectory();
  
  // RedRightNoCollect
  public static Trajectory RedRightNoCollect1 = new Trajectory();
  
  public static Trajectory RedRightNoCollect = new Trajectory();
  
  // Blue Left Charge
  public static Trajectory BlueLeftCharge1 = new Trajectory();
  public static Trajectory BlueLeftCharge2 = new Trajectory();

  public static Trajectory BlueLeft = new Trajectory();

  // Blue Left No Charge
  public static Trajectory BlueLeftNoCharge1 = new Trajectory();
  public static Trajectory BlueLeftNoCharge2 = new Trajectory();
  public static Trajectory BlueLeftNoCharge3 = new Trajectory();

  public static Trajectory BlueLeftNoCharge = new Trajectory();

    // Blue Left No Collect
    public static Trajectory BlueLeftNoCollect1 = new Trajectory();
  
    public static Trajectory BlueLeftNoCollect = new Trajectory();

    public static CollectorState collectorState = CollectorState.EMPTY;




  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(15)); 
    autoChooser.addOption("Red Left (No Charge)", new RedLeftNoCharge());
    autoChooser.addOption("Red Right (score 2 then engage)", new RedRight2AndCharge());
    autoChooser.addOption("Red Right 2.5 and balance", new RedRight3AndEngage());
    autoChooser.addOption("Red Right 2 Cone (score 3 no engage)", new RedRight3NoCharge2Cube()); // 2 Cones
    autoChooser.addOption("Red Right score 3 mid cube", new RedRightScore3Cube2()); //TODO: Needs to be created, Red Right 2 Cone Scores 3
    autoChooser.addOption("Blue Left (score 2 then engage)", new BlueLeft2AndCharge());
    autoChooser.addOption("Blue Left 2 Cone (score 3 no engage)", new BlueLeft3NoCharge2Cube()); // 2 Cones
    autoChooser.addOption("Blue left score 3 mid cube", new BlueLeftScore3Cube2()); // 2 Cubes
    autoChooser.addOption("Blue Left 2.5 and balance", new BlueLeft3AndEngage()); //
    autoChooser.addOption("Blue Right (No Charge)", new BlueRightNoCharge()); 
    autoChooser.addOption("TEST Score Cone High", new ScoreConeHighAndWait());
    

    //Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
    Robot.leds.colorTest(255, 30, 0);
    //Robot.leds.solid(Color.kOrange);
    Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
    Robot.drivetrain.setDefaultCommand(new BobDrive());
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    String RedRightCharge1JSON = "paths//redrightcharge1.wpilib.json";
    String RedRightCharge2JSON = "paths//redrightcharge2.wpilib.json";

    String RedRightNoCharge1JSON = "paths//redrightnocharge1.wpilib.json";
    String RedRightNoCharge2JSON = "paths//redrightnocharge2.wpilib.json";
    String RedRightNoCharge3Json = "paths//redrightnocharge3.wpilib.json";

    String RedRightNoCollect1Json = "paths//redrightnocollect1.wpilib.json";

    String BlueLeftCharge1JSON = "paths//blueleftcharge1.wpilib.json";
    String BlueLeftCharge2JSON = "paths//blueleftcharge2.wpilib.json";

    String BlueLeftNoCharge1JSON = "paths//blueleftnocharge1.wpilib.json";
    String BlueLeftNoCharge2JSON = "paths//blueleftnocharge2.wpilib.json";
    String BlueLeftNoCharge3JSON = "paths//blueleftnocharge3.wpilib.json";
    
    String BlueLeftNoCollect1JSON = "paths//blueleftnocollect1.wpilib.json";
 
    try {

      //Blue Left Charge

      Path BlueLeftChargePath1 = Filesystem.getDeployDirectory().toPath().resolve(BlueLeftCharge1JSON);
      Path BlueLeftChargePath2 = Filesystem.getDeployDirectory().toPath().resolve(BlueLeftCharge2JSON);

      BlueLeftCharge1 = TrajectoryUtil.fromPathweaverJson(BlueLeftChargePath1);
      BlueLeftCharge2 = TrajectoryUtil.fromPathweaverJson(BlueLeftChargePath2);

      BlueLeft = BlueLeftCharge1.concatenate(BlueLeftCharge2);

      //Blue Left No Charge

      Path BlueLeftNoChargePath1 = Filesystem.getDeployDirectory().toPath().resolve(BlueLeftNoCharge1JSON);
      Path BlueLeftNoChargePath2 = Filesystem.getDeployDirectory().toPath().resolve(BlueLeftNoCharge2JSON);
      Path BlueLeftNoChargePath3 = Filesystem.getDeployDirectory().toPath().resolve(BlueLeftNoCharge3JSON);

      BlueLeftNoCharge1 = TrajectoryUtil.fromPathweaverJson(BlueLeftNoChargePath1);
      BlueLeftNoCharge2 = TrajectoryUtil.fromPathweaverJson(BlueLeftNoChargePath2);
      BlueLeftNoCharge3 = TrajectoryUtil.fromPathweaverJson(BlueLeftNoChargePath3);

      BlueLeftNoCharge = BlueLeftNoCharge1.concatenate(BlueLeftNoCharge2);
      BlueLeftNoCharge = BlueLeftNoCharge.concatenate(BlueLeftNoCharge3);

      //BlueLeftNoCollect
      Path BlueLeftNoCollectPath1 = Filesystem.getDeployDirectory().toPath().resolve(BlueLeftNoCollect1JSON);

      BlueLeftNoCollect1 = TrajectoryUtil.fromPathweaverJson(BlueLeftNoCollectPath1);

      BlueLeftNoCollect = BlueLeftNoCollect1;

      //RedRightCharge
      Path RedRightChargePath1 = Filesystem.getDeployDirectory().toPath().resolve(RedRightCharge1JSON);
      Path RedRightChargePath2 = Filesystem.getDeployDirectory().toPath().resolve(RedRightCharge2JSON);
      

      RedRightCharge1 = TrajectoryUtil.fromPathweaverJson(RedRightChargePath1);
      RedRightCharge2 = TrajectoryUtil.fromPathweaverJson(RedRightChargePath2);
      

      RedRight = RedRightCharge1.concatenate(RedRightCharge2);

      //RedRightNoCharge
      Path RedRightNoChargePath1 = Filesystem.getDeployDirectory().toPath().resolve(RedRightNoCharge1JSON);
      Path RedRightNoChargePath2 = Filesystem.getDeployDirectory().toPath().resolve(RedRightNoCharge2JSON);
      Path RedRightNoChargePath3 = Filesystem.getDeployDirectory().toPath().resolve(RedRightNoCharge3Json);

      RedRightNoCharge1 = TrajectoryUtil.fromPathweaverJson(RedRightNoChargePath1);
      RedRightNoCharge2 = TrajectoryUtil.fromPathweaverJson(RedRightNoChargePath2);
      RedRightNoCharge3 = TrajectoryUtil.fromPathweaverJson(RedRightNoChargePath3);

      RedRightNoChargeFirstPath = RedRightNoCharge1.concatenate(RedRightNoCharge2);
      RedRightNoChargeFirstPath = RedRightNoChargeFirstPath.concatenate(RedRightNoCharge3);

      //BlueLeftNoCollect
      Path RedRightNoCollectPath1 = Filesystem.getDeployDirectory().toPath().resolve(RedRightNoCollect1Json);

      RedRightNoCollect1 = TrajectoryUtil.fromPathweaverJson(RedRightNoCollectPath1);

      RedRightNoCollect = RedRightNoCollect1;

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
    //Robot.leds.solidRGB(Section.FULL, 0x00, 0x00, 0x00);
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    //SmartDashboard.putNumber("Collector State", collectorState);
    
    SmartDashboard.putNumber("Elevator Position", elevator.getCurrentPosition());
    SmartDashboard.putNumber("Elevator Velocity", elevator.getVelocity());
    SmartDashboard.putNumber("Elevator Current", elevator.getElevatorCurrent());

    SmartDashboard.putNumber("Elbow Position", elbow.getCurrentPosition());
    SmartDashboard.putNumber("Elbow Current", elbow.getElbowCurrent());
    SmartDashboard.putNumber("Elbow Velocity", elbow.getElbowMotorVelocity());

    SmartDashboard.putNumber("Wrist Position", wrist.getCurrentPosition());
    SmartDashboard.putNumber("Wrist Current", wrist.getWristCurrent());
    SmartDashboard.putNumber("Wrist Velocity", wrist.getWristMotorVelocity());

    SmartDashboard.putNumber("Left Encoder Distance", drivetrain.getLeftLeadDriveDistanceMeters());
    SmartDashboard.putNumber("Right Encoder Distance", drivetrain.getRightLeadDriveDistanceMeters());
    SmartDashboard.putNumber("Heading", drivetrain.getHeadingDegrees());

    SmartDashboard.putNumber("Limelight Distance", limelight.getDistanceToTarget(Constants.TargetType.CONE_HIGH));
    SmartDashboard.putNumber("pitch",drivetrain.getPitch());

    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(testAutoChooser);
    
   
    //drivetrain.setFollowers();

    CommandScheduler.getInstance().run();
    
    //Robot.leds.strobe(Section.FULL,Color.kOrange, 0.1);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Robot.elbow.elbowMotor.setIdleMode(IdleMode.kCoast);
    Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void disabledPeriodic() {
    //Robot.elevator.setPosition(Robot.elevator.getCurrentPosition());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    collectorState = CollectorState.HOLDING_CONE; // We always hold a cone first

    Robot.drivetrain.setNeutralMode(NeutralMode.Brake);

    m_autonomousCommand = autoChooser.getSelected();
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
    Robot.elevator.setPosition(Robot.elevator.getCurrentPosition());
    Robot.elbow.setPosition(Robot.elbow.getCurrentPosition());
    Robot.wrist.setPosition(Robot.wrist.getCurrentPosition());
    Robot.elbow.elbowMotor.setIdleMode(IdleMode.kCoast);

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

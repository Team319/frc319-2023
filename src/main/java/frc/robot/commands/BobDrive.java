package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.utils.DriveMode;
import frc.robot.utils.DriveSignal;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.BobDriveHelper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class BobDrive extends CommandBase {

	BobDriveHelper helper;
	private double quickTurnThreshold = 0.2;
	private double deadband = 0.1;

	private RobotContainer robotContainer = new RobotContainer();
	//private PIDController limelightRotatePID = new PIDController(0.25, 0.01, 0.0);

	public BobDrive() {
		addRequirements(Robot.drivetrain);
		helper = new BobDriveHelper();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {

		double rotateValue = robotContainer.getRightStick().getFirst() * 0.50;
		double moveValue = robotContainer.getLeftStick().getSecond() * 1.00;

		moveValue = Math.abs(moveValue) > deadband ? moveValue : 0.0;
		rotateValue = Math.abs(rotateValue) > deadband ? rotateValue : 0.0;

		boolean quickTurn = (moveValue < quickTurnThreshold && moveValue > -quickTurnThreshold);	
		DriveSignal driveSignal = helper.cheesyDrive(moveValue, rotateValue, quickTurn, false);
		Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
	}

	public boolean isFinished() {
		return false;
	}

	public void end() {
	}

	public void interrupted() {
	}
}

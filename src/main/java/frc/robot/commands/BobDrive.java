package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.utils.DriveSignal;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.utils.BobDriveHelper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class BobDrive extends CommandBase {

	BobDriveHelper helper;
	private double quickTurnThreshold = 0.2;
	private double deadband = 0.1;
	private double rotateValue = 0.0;
	private double moveValue = 0.0;
	private PIDController limelightRotatePID = new PIDController(0.25, 0.01, 0.0); // TODO: Tune me.

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
		if (Robot.drivetrain.getDriveMode() == DriveMode.Limelight) {
			rotateValue = limelightRotatePID.calculate(Robot.limelight.getXProportional());
		}
		else {
			rotateValue = robotContainer.getRightStick().getFirst() * 0.40;
		}

		SmartDashboard.putNumber("rotate value", rotateValue);
		SmartDashboard.putNumber("bob drivemode", Robot.drivetrain.getDriveMode().ordinal());

		moveValue = robotContainer.getLeftStick().getSecond() * 1.0;
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

}

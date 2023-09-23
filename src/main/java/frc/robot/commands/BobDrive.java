package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.utils.DriveSignal;
import frc.robot.utils.HelperFunctions;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants.DriveMode;
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
	private double rotateValue;
	private double stickModifier;
	private double moveValue = 0.0;
	private PIDController cubeLimelightRotatePID = new PIDController(0.75, 0.08, 0.0); // TODO: Tune me.
	private PIDController scoringLimelightRotatePID = new PIDController(0.5, 0.08, 0.0); // TODO: Tune me.

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
		rotateValue = 0.0;
		if (Robot.drivetrain.getDriveMode() == DriveMode.Limelight) {

			// TODO - check which pipeline we're in
			// If we're in pipeline 3 ( cube ) use the "collect" table results
			if(Robot.limelight.getPipelineCollect() == 3) {
				if(Robot.limelight.getVCollect() != 0) {
					stickModifier = HelperFunctions.deadband(robotContainer.getRightStick().getFirst(), deadband) * 0.5;
					rotateValue = -cubeLimelightRotatePID.calculate(Robot.limelight.getXProportionalCollect()+ stickModifier );
				}
				else {
					rotateValue = HelperFunctions.deadband(robotContainer.getRightStick().getFirst(), deadband) * 0.20;
				}
			}
			//otherwise use the existing
			if(Robot.limelight.getPipeline() == 1 || Robot.limelight.getPipeline() == 2  ){

			
			if(Robot.limelight.getV() != 0){
				stickModifier = HelperFunctions.deadband(robotContainer.getRightStick().getFirst(), deadband) * 0.5;
				rotateValue = -scoringLimelightRotatePID.calculate(Robot.limelight.getXProportional()+ stickModifier ) ;
			} else {
				rotateValue = HelperFunctions.deadband(robotContainer.getRightStick().getFirst(), deadband) * 0.20;
			}
		}

			moveValue = HelperFunctions.deadband(robotContainer.getLeftStick().getSecond(), deadband)* 0.4;
		}
		else if (Robot.drivetrain.getDriveMode() == DriveMode.Normal) {
			rotateValue = HelperFunctions.deadband(robotContainer.getRightStick().getFirst(), deadband) * 0.30;
			moveValue = HelperFunctions.deadband(robotContainer.getLeftStick().getSecond(), deadband);
		}
		else if (Robot.drivetrain.getDriveMode() == DriveMode.Scoring) {
			rotateValue = HelperFunctions.deadband(robotContainer.getRightStick().getFirst(), deadband) * 0.20;
			moveValue = HelperFunctions.deadband(robotContainer.getLeftStick().getSecond(), deadband) * 0.4;
		}

		//SmartDashboard.putNumber("rotate value", rotateValue);
		//SmartDashboard.putNumber("bob drivemode", Robot.drivetrain.getDriveMode().ordinal());

		

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

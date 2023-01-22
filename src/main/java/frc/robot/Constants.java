// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static class DriveConstants {
      /* Taken from WPILib Docs - L.S
      for *your* robot's drive.
      The Robot Characterization Toolsuite provides a convenient tool for obtaining these
      These characterization values MUST be determined either experimentally or theoretically
      values for your robot.*/

      public static final double ksVolts = 0.080151; 
      public static final double kvVoltSecondsPerMeter = 3.895;
      public static final double kaVoltSecondsSquaredPerMeter = 0.20594; 

      // Example value only - as above, this must be tuned for your drive!
      public static final double kPDriveVel = 0.080207;

      public static final double kTrackWithMeters = 0.71; // roughly
      // DifferentialDrive Kinematics

      public static final double kMaxSpeedMetersPerSecond = 1.5; // was 2.5
      public static final double kMaxAccerlationMetersPerSecondSquared = 0.5; // was 2.5

      // Taken from WPILib Docs - L.S
      // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;

      public static final double encoderTicksPerRev = 2048.0;
      public static final double wheelRotPerMotorRot = 1.0/10.0;
      public static final double wheelDiameter = 3.5; //inches
      public static final double metersPerWheelRot = Units.inchesToMeters(wheelDiameter * Math.PI);
      public static final double metersPerEncoderTick = (1/encoderTicksPerRev) * wheelRotPerMotorRot * metersPerWheelRot;

      public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWithMeters);

    }
  
  
    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}

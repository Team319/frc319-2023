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
      public enum DriveMode {
        Normal, Limelight,
    }
    

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

    public static class ElevatorConstants {  // Gear Ratio: 5/1(?)
      public static class PID {
        public static final double kPUp = 0.9;
        public static final double kIUp = 0;
        public static final double kDUp = 0;

        public static final double kPDown = 0.1;
        public static final double kIDown = 0;
        public static final double kDDown = 0;

        public static final int iZone = 0;
        
        public static final double fGainUp = 0.00019231;
        public static final double fGainDown = 0;
      }

      public static class SetPoints {
        public static final double scoreHighCone = 0;
        public static final double scoreMiddleCone = 0;

        public static final double scoreHighCube = 0;
        public static final double scoreMiddleCube = 0;
        
        public static final double scoreFloor = 0;
        
        public static final double home = 0.75;
        public static final double top = 55.0;
        public static final double middle = 35.0;

        public static final double deadband = 1.5;
      }

      public static class SoftLimits {
        public static final double forwardSoftLimit = 55.0;
        public static final double reverseSoftLimit = 0.75;
      }
    }

    public static class ElbowConstants {
      public static class PID {
        public static final double kUpP = 0.106155;
        public static final double kUpI = 0.00106155;
        public static final double kUpD = 0;

        public static final double kDownP = 0.038462;
        public static final double kDownI = 0;
        public static final double kDownD = 0;

        public static final double iZone = 0.3;
        public static final double fGain = 0.00019231;
      }

      public static class SetPoints {
        public static final double scoreHighCone = 0;
        public static final double scoreMiddleCone = 0;

        public static final double scoreHighCube = 0;
        public static final double scoreMiddleCube = 0;

        public static final double scoreFloor = 0;

        public static final double home = 0;
        public static final double top = Constants.ElbowConstants.SoftLimits.forwardSoftLimit;
        public static final double bottom = Constants.ElbowConstants.SoftLimits.reverseSoftLimit;

        public static final double deadband = 1.5;
      }

      public static class SoftLimits {
        public static final double forwardSoftLimit = 23.4;
        public static final double reverseSoftLimit = -39.1;
      }
    }

    public static class WristConstants {
      public static class PID {
        public static final double kP = 0.096155;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final int iZone = 0;
        public static final double fGain = 0.00019231;
      }

      public static class SetPoints {
        public static final double scoreHighCone = 0;
        public static final double scoreMiddleCone = 0;

        public static final double scoreHighCube = 0;
        public static final double scoreMiddleCube = 0;

        public static final double scoreFloor = 0;

        public static final double home = 0;
        public static final double top = Constants.WristConstants.SoftLimits.forwardSoftLimit;
        public static final double bottom = Constants.WristConstants.SoftLimits.reverseSoftLimit;

        public static final double deadband = 0.5;
      }

      public static class SoftLimits {
        public static final double forwardSoftLimit = 0.0;
        public static final double reverseSoftLimit = -103.0;
      }
    }

    public static class LimelightConstants {
      public static class Modes {
        public static final int APRIL_TAG_MODE = 0;
        public static final int LIMELIGHT_BOTTOM = 1;
        public static final int LIMELIGHT_TOP = 2;
      }
      
      public static final int LED_ON = 3;
      public static final int LED_OFF = 1;
    }
  
  
    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
}

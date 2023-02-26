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
        Normal, Limelight, Scoring
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
        public static final double kPUp = 0.5;
        public static final double kIUp = 0;
        public static final double kDUp = 1.0;

        public static final double kPDown = 0.1;
        public static final double kIDown = 0;
        public static final double kDDown = 1.0;

        public static final int iZone = 0;
        
        public static final double fGainUp = 0.00019231;
        public static final double fGainDown = 0;
      }

      public static class SetPoints {
        public static final double scoreHighCone = 64;
        public static final double scoreMiddleCone = 40.78;

        public static final double scoreHighCube = 0;
        public static final double scoreMiddleCube = 0;
        
        public static final double scoreFloor = 0;

        public static final double preScore = 19;

        public static final double collectTipped = 19;
        public static final double collectStanding = 8;
        public static final double collectFloor = 10.78;
        public static final double collectCubeFromLoadStation = 52;
        public static final double collectConeFromLoadStation = 60;
        
        public static final double home = 0.75;
        public static final double top = 55.0;
        public static final double middle = 35.0;

        public static final double deadband = 1.5;
      }

      public static class SoftLimits {
        public static final double forwardSoftLimit = 66.0;
        public static final double reverseSoftLimit = 0.75;
      }

      public static class Currents {
        public static final int currentMax = 20;
        public static final int currentThreshold = 0;
      }

      public static class SmartMotionParameters {
        public static final int smartMotionSlot = 0;
        public static final double maxVel = 5200.0;
        public static final double minVel = 0.0;
        public static final double maxAccel = 1000.0;
        public static final double maxErr = 100.0;
      }
    }

    public static class ElbowConstants {
      public static class PID {
        public static final double kUpP = 5.5;
        public static final double kUpI = 0.018;
        public static final double kUpD = 0;

        public static final double kDownP = 2.0;
        public static final double kDownI = 0.018;
        public static final double kDownD = 0;

        public static final double iZone = 0.005;
        public static final double fGain = 0.018;
      }

      public static class SetPoints {
        public static final double scoreHighCone = 0.197700;
        public static final double scoreMiddleCone = 0.197700;

        public static final double scoreHighCube = 0.0;
        public static final double scoreMiddleCube = 0.0;

        public static final double scoreFloor = 0.0;

        public static final double preScore = 0.197700;

        public static final double collectConeTipped = -0.148682;
        public static final double collectConeStanding = 0.0;
        public static final double collectFloor = 0.0;
        public static final double collectFromLoadStation = 0.216919;


        public static final double home = 0.0;
        public static final double top = 0.193970;
        public static final double bottom = -0.208496;
        public static final double testbottom= -0.174316;

        public static final double deadband = 0.05;
      }

      public static class SoftLimits {
        public static final double forwardSoftLimit = 0.193970;
        public static final double reverseSoftLimit = -0.208496;
      }

      public static class Currents {
        public static final int currentMax = 20;
        public static final int currentThreshold = 0;
      }

      public static class SmartMotionParameters {
        public static final int smartMotionSlot = 0;
        public static final double maxVel = 55.0;
        public static final double minVel = 0.0;
        public static final double maxAccel = 10.0;
        public static final double maxErr = 10.0;
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
        public static final double scoreHighCone = -60;
        public static final double scoreMiddleCone = -66;

        public static final double scoreHighCube = 0;
        public static final double scoreMiddleCube = 0;

        public static final double scoreFloor = 0;

        public static final double preScore = -33;

        public static final double collectConeTipped = -63;
        public static final double collectConeStanding = -60;
        public static final double collectFloor = -75;
        public static final double collectFromLoadStation = -58;

        public static final double home = -31;
        public static final double top = 0.0;
        public static final double bottom = -103.0;

        public static final double deadband = 0.5;
      }

      public static class SoftLimits {
        public static final double forwardSoftLimit = 0.0;
        public static final double reverseSoftLimit = -103.0;
      }

      public static class Currents {
        public static final int currentMax = 20;
        public static final int currentThreshold = 0;
      }

      public static class SmartMotionParameters {
        public static final int smartMotionSlot = 0;
        public static final double maxVel = 100.0;
        public static final double minVel = 10.0;
        public static final double maxAccel = 10.0;
        public static final double maxErr = 100.0;
      }
    }

    public static class CollectorConstants {
      
      public static class Currents {
        public static final int currentMax = 30;
        public static final int currentThreshold = 15;

        public static final double collectorVoltage = 0.75;
      }

      public static class PID {
        public static final double kP = 0.00019231;
        public static final double fGain = 0.00019231;
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

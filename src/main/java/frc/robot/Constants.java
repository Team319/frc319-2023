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
    
                                                                      // These values worked
      public static final double ksVolts = 0.025;                      // 0.05
      public static final double kvVoltSecondsPerMeter = 0.2;           // 2.0594;
      public static final double kaVoltSecondsSquaredPerMeter = 0.0002; // 0.01

      // Example value only - as above, this must be tuned for your drive!
      public static final double kPDriveVel = 0.0008;  //Testing : 0.0008;//sysId : 0.9146;

      public static final double kTrackWithMeters = 0.64; // roughly
      // DifferentialDrive Kinematics

      public static final double kMaxSpeedMetersPerSecond = 4.5; // was 2.5
      public static final double kMaxAccerlationMetersPerSecondSquared = 1; // was 2.5

      // Taken from WPILib Docs - L.S
      // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;

      public static final double encoderTicksPerRev = 2048.0;
      public static final double wheelRotPerMotorRot = 1.0/6.25;
      public static final double wheelDiameter = 4.0; //inches
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
        
        public static final double fGainUp = 0.00017544;//0.00019231;
        public static final double fGainDown = 0;
      }

      public static class SetPoints {
        public static final double scoreHighCone = 65.3;
        public static final double scoreMiddleCone = 42.0; // 45.0

        public static final double autoScoreHighCube = 39.0;
        public static final double scoreMiddleCube = 0;
        
        public static final double scoreFloor = 0;

        public static final double preScore = 19.43;

        public static final double autoStartPosition = 0.0;
        public static final double autoCollectConeTipped = 0.0;
        public static final double collectConeTipped = 0.0; // was 20
        public static final double collectStanding = 0.0; // was 8.0
        public static final double collectFloor = 0.0;  // was 10.78
        public static final double coneSlide = 18.285;
        public static final double coneSingleLoadingStation = 0.0;
        public static final double collectCubeFromLoadStation = 52.0;
        public static final double collectConeFromLoadStation = 58.0;
        
        public static final double home = 0.0;
        public static final double top = 55.0;
        public static final double middle = 35.0;

        public static final double deadband = 1.5;
      }

      public static class SoftLimits {
        public static final double forwardSoftLimit = 66.0;
        public static final double reverseSoftLimit = 0.0;
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
        public static final double kUpP = 9.5;
        public static final double kUpI = 0.018;
        public static final double kUpD = 0;

        public static final double kDownP = 6.5;
        public static final double kDownI = 0.018;
        public static final double kDownD = 0;

        public static final double iZone = 0.005;
        public static final double fGain = 0.018;
      }

      public static class SetPoints {
        public static final double scoreHighCone = -0.0548;
        public static final double scoreMiddleCone = -0.0548;

        public static final double autoScoreHighCube = 0.088013;
        public static final double scoreMiddleCube = 0.0;

        public static final double scoreFloor = 0.0;

        public static final double preScore = -0.070;

        public static final double autoStartPosition = 0.0;
        public static final double autoCollectConeTipped = -0.48;
        public static final double collectConeTipped = -0.535;
        public static final double collectConeStanding = -0.451;
        public static final double collectFloor = -0.468;
        public static final double coneSlide = -0.486;
        public static final double coneSingleLoadingStation = 0.0;
        public static final double collectFromLoadStation = 0.216919;


        public static final double home = -0.070;
        public static final double top = 0.193970;
        public static final double bottom = -0.208496;
        public static final double testbottom= -0.174316;

        public static final double deadband = 0.05;
      }

      public static class SoftLimits {
        public static final double forwardSoftLimit = 0.0;
        public static final double reverseSoftLimit = -0.6;
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
        public static final double kP = 10.0;//0.096155;
        public static final double kI = 0.018;//0;
        public static final double kD = 0;

        public static final double iZone = 0.002;//0;
        public static final double fGain = 0.018;//0.00019231;
      }

      public static class SetPoints {
        public static final double scoreHighCone = 0.438; //-60;
        public static final double scoreMiddleCone = 0.438; 

        public static final double autoScoreHighCube = -0.283; //-26;
        //public static final double scoreMiddleCube = 0;

        //public static final double scoreFloor = 0;

        public static final double preScore = 0.263; //-33;

        public static final double autoStartPosition = 0.0;
        public static final double autoCollectConeTipped = 0.198;
        public static final double collectConeTipped = 0.3220; 
        public static final double collectConeStanding = 0.215; 
        public static final double coneSlide = 0.458;
        public static final double collectFloor = 0.134;
        public static final double coneSingleLoadingStation = 0.0;
        public static final double collectFromLoadStation = -0.54;//-58;

        public static final double home = 0.263;//-27;
        //public static final double top = 0.0;
        public static final double bottom = -0.6;

        public static final double deadband = 0.005;
      }

      public static class SoftLimits {
        public static final double forwardSoftLimit = 0.65;
        public static final double reverseSoftLimit = 0.0;
        //public static final double forwardSoftLimit = 0.0;
        //public static final double reverseSoftLimit = -103.0;
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
        public static final int currentMax = 20;
        public static final int currentThreshold = 15;

        public static final double collectorVoltage = 0.75;
        public static final double collectorVoltageTipped = 0.5;
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
      
      public static final double mountAngleDegrees = 10.0;
      public static final double LensHeightInches = 21.0; // ish - EKM 3/3/23\

      public static final double highConeTargetHeight = 44.0; 
      public static final double midConeTargetHeight = 24.0; 
      public static final double aprilTagTargetHeight = 10.0;

      public static final int LED_ON = 3;
      public static final int LED_OFF = 1;
    }
  
  
    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  //TODO - Make this an enum
  public static class TargetType {
    public static final int FLOOR = 0;
    public static final int APRILTAG = 1;
    public static final int CONE_MID = 2;
    public static final int CONE_HIGH = 3;
  }

  public enum CollectorState {
    EMPTY, HOLDING_CUBE, HOLDING_CONE
}

}

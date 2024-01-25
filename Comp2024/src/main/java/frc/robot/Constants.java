
package frc.robot;

import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants
{
  // Toggles constants between comp robot and practice robot (named "beta")
  public static boolean      isComp;

  // bot serial nums
  public static final String kCompSN               = "03238074";
  public static final String kBetaSN               = "03260A3A";

  // Game controller definitions
  public static final int    kDriverPadPort        = 0;
  public static final int    kOperatorPadPort      = 1;

  public static final double kStickDeadband        = 0.15;
  public static final double kTriggerThreshold     = 0.25;

  // Phoenix firmware versions expected
  public static final int    kPhoenix5MajorVersion = ((22 * 256) + 0);
  public static final int    kPhoenix6MajorVersion = 23;

  public static final double kAutonomousPeriodSecs = 15.0;

  /////////////////////////////////////////////////////////////////////////////
  // CAN IDs and PWM IDs
  /////////////////////////////////////////////////////////////////////////////
  public static final class Ports
  {
    public static final String kCANCarnivore         = "canivore1";
    public static final String kCANRio               = "rio";

    // CANivore CAN IDs - Swerve
    public static final int    kCANID_DriveLF        = 1;    // Falcon 500
    public static final int    kCANID_SteerLF        = 2;    // Falcon 500
    public static final int    kCANID_CANCoderLF     = 3;    // CANCoder

    public static final int    kCANID_DriveRF        = 4;    //Falcon 500     
    public static final int    kCANID_SteerRF        = 5;    // Falcon 500
    public static final int    kCANID_CANCoderRF     = 6;    // CANCoder

    public static final int    kCANID_DriveLR        = 7;    //Falcon 500
    public static final int    kCANID_SteerLR        = 8;    // Falcon 500
    public static final int    kCANID_CANCoderLR     = 9;    // CANCoder

    public static final int    kCANID_DriveRR        = 10;   //Falcon 500
    public static final int    kCANID_SteerRR        = 11;   // Falcon 500
    public static final int    kCANID_CANCoderRR     = 12;   // CANCoder

    public static final int    kCANID_Pigeon2        = 13;   // Pigeon2 IMU

    // RoboRIO CAN IDs
    public static final int    kCANID_IntakeRoller   = 15;   // Talon SRX - 775Pro
    public static final int    kCANID_IntakeRotary   = 16;   // Falcon 500
    public static final int    kCANID_IntakeCANCoder = 17;   // CANCoder

    public static final int    kCANID_FeederRoller   = 19;   // Talon SRX - 775Pro
    public static final int    kCANID_FeederRotary   = 20;   // Falcon 500
    public static final int    kCANID_FeederCANCoder = 21;   // CANCoder

    public static final int    kCANID_ShooterL       = 23;   // Falcon 500
    public static final int    kCANID_ShooterR       = 24;   // Falcon 500
    public static final int    kCANID_ShooterRotary  = 25;   // Falcon 500

    public static final int    kCANID_ClimberL       = 27;   // Falcon 500
    public static final int    kCANID_ClimberR       = 28;   // Falcon 500

    public static final int    kCANID_CANdle         = 0;

    // Digital I/Os
    // public static final int    kDIO_ExampleDetect = 2;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Falcon 500
  /////////////////////////////////////////////////////////////////////////////
  public static final class Falcon500
  {
    public static int          kMaxRPM     = 6380; // free speed for Falcon 500 motor
    public static final double kEncoderCPR = 2048; // CPR is from Falcon 500 Manual
  }

  /////////////////////////////////////////////////////////////////////////////
  // Swerve drive
  /////////////////////////////////////////////////////////////////////////////
  public static final class SWConsts
  {
    /* Individual module constants */
    public static final double               driveGearRatio                = 6.75;
    public static final double               steerGearRatio                = 21.43;

    /* Swerve Current Limiting */
    public static final int                  driveSupplyCurrentLimit       = 35;
    public static final int                  driveSupplyCurrentThreshold   = 60;
    public static final double               driveSupplyTimeThreshold      = 0.1;
    public static final boolean              driveSupplyCurrentLimitEnable = true;

    public static final int                  steerSupplyCurrentLimit       = 25;
    public static final int                  steerSupplyCurrentThreshold   = 40;
    public static final double               steerSupplyTimeThreshold      = 0.1;
    public static final boolean              steerSupplyCurrentLimitEnable = true;

    /* Drive Motor PID Values */
    public static final double               driveFFKS                     = 0.0;
    public static final double               driveFFKV                     = 0.0;
    public static final double               driveKP                       = 0.36;
    public static final double               driveKI                       = 0.0;
    public static final double               driveKD                       = 0.0;

    /* Steering Motor PID Values */
    public static final double               steerFFKS                     = 0.0;
    public static final double               steerFFKV                     = 0.0;
    public static final double               steerKP                       = 12.0;
    public static final double               steerKI                       = 0.0;
    public static final double               steerKD                       = 0.0;

    /* Neutral Modes */
    public static final InvertedValue        driveMotorInvert              = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue     driveNeutralMode              = NeutralModeValue.Brake;
    public static final double               driveOpenLoopRamp             = 0.25;
    public static final double               driveClosedLoopRamp           = 0.0;

    public static final InvertedValue        steerMotorInvert              = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue     steerNeutralMode              = NeutralModeValue.Coast;
    public static final double               steerOpenLoopRamp             = 0.0;
    public static final double               steerClosedLoopRamp           = 0.0;

    public static final SensorDirectionValue steerCanCoderInvert           = SensorDirectionValue.CounterClockwise_Positive;
    public static final boolean              gyroInvert                    = false; // Always ensure Gyro is CCW+ CW-

    /* Drive Motor Characterization Values */
    public static final double               driveKS                       = (0.32 / 12);
    public static final double               driveKV                       = (1.51 / 12);
    public static final double               driveKA                       = (0.27 / 12);

    /* Swerve Profiling Values */
    public static final double               maxSpeed                      = 4.5;  // meters per second
    public static final double               maxAngularVelocity            = 6.0;  // orginially 10.0
    public static final double               maxSpeedSlowMode              = 2.25; // meters per second
    public static final double               maxAngularVelocitySlowMode    = 4.0;  // orginially 5.0

    /* Controller Invert */
    public static final boolean              invertXAxis                   = false;
    public static final boolean              invertYAxis                   = false;
    public static final boolean              invertRAxis                   = false;

    /*** MODULE SPECIFIC CONSTANTS ***/

    /* Front Left Module - Module 0 */
    public static final class Mod0
    {
      public static final double betaAngleOffset = 0.0;
      public static final double compAngleOffset = 0.043945;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveLF, Ports.kCANID_SteerLF, Ports.kCANID_CANCoderLF,
            isComp ? compAngleOffset : betaAngleOffset);
      }
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1
    {
      public static final double betaAngleOffset = 0.0;
      public static final double compAngleOffset = -0.331787;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveRF, Ports.kCANID_SteerRF, Ports.kCANID_CANCoderRF,
            isComp ? compAngleOffset : betaAngleOffset);
      }
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2
    {
      public static final double betaAngleOffset = 0.0;
      public static final double compAngleOffset = 0.272949;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveLR, Ports.kCANID_SteerLR, Ports.kCANID_CANCoderLR,
            isComp ? compAngleOffset : betaAngleOffset);
      }
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3
    {
      public static final double betaAngleOffset = 0.0;
      public static final double compAngleOffset = 0.25836;

      public static SwerveModuleConstants SwerveModuleConstants( )
      {
        return new SwerveModuleConstants(Ports.kCANID_DriveRR, Ports.kCANID_SteerRR, Ports.kCANID_CANCoderRR,
            isComp ? compAngleOffset : betaAngleOffset);
      }
    }

    /*** SWERVE SUBSYSTEM CONSTANTS ***/

    public static final double                trackWidth            = Units.inchesToMeters(22.7);
    public static final double                wheelBase             = Units.inchesToMeters(22.7);

    public static final double                wheelDiameter         = Units.inchesToMeters(4.0);
    public static final double                wheelCircumference    = wheelDiameter * Math.PI;

    public static final Translation2d[ ]      swerveModuleLocations =
    {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0), new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    };

    public static final SwerveDriveKinematics swerveKinematics      = new SwerveDriveKinematics(swerveModuleLocations);

  }

  /////////////////////////////////////////////////////////////////////////////
  // Swerve snap constraints
  /////////////////////////////////////////////////////////////////////////////
  public static final class SnapConsts
  {
    public static final double                       kP                                      = 5.0;
    public static final double                       kI                                      = 0;
    public static final double                       kD                                      = 0.0;
    public static final double                       kTimeout                                = 0.75;
    public static final double                       kEpsilon                                = 1.0;

    // Constraints for the profiled angle controller
    public static final double                       kMaxAngularSpeedRadiansPerSecond        = 2.0 * Math.PI;
    public static final double                       kMaxAngularSpeedRadiansPerSecondSquared =
        Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints             =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Autonomous trajectory constraints
  /////////////////////////////////////////////////////////////////////////////
  public static final class AutoConsts
  {
    public static final double                       kMaxAngularSpeedRadiansPerSecond        = 1.2 * Math.PI;
    public static final double                       kMaxAngularSpeedRadiansPerSecondSquared =
        Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

    public static final double                       kPXController                           = 1;
    public static final double                       kPYController                           = 1;
    public static final double                       kPThetaController                       = 5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints             =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  }

  /////////////////////////////////////////////////////////////////////////////
  // Intake subsystem
  /////////////////////////////////////////////////////////////////////////////
  public static final class INConsts
  {
    // Global settings

    // Current limit settings - Roller
    // Current limit settings - Rotary

    // Manual config parameters
    public enum RollerMode
    {
    }

    // Manual config parameters
    public enum RotaryMode
    {
    }

    // Motion Magic config parameters
  }

  /////////////////////////////////////////////////////////////////////////////
  // Feeder subsystem 
  /////////////////////////////////////////////////////////////////////////////
  public static final class FDConsts
  {
    // Global settings

    // Current limit settings - Roller
    // Current limit settings - Rotary

    // Manual config parameters
    public enum RollerMode
    {
    }

    // Manual config parameters
    public enum RotaryMode
    {
    }

    // Motion Magic config parameters
  }

  /////////////////////////////////////////////////////////////////////////////
  // Shooter subsystem 
  /////////////////////////////////////////////////////////////////////////////
  public static final class SHConsts
  {
    // Global settings

    // Current limit settings - Roller
    // Current limit settings - Rotary

    // Manual config parameters
    public enum ShooterMode
    {
    }

    // Manual config parameters
    public enum RotaryMode
    {
    }

    // Motion Magic config parameters
  }

  /////////////////////////////////////////////////////////////////////////////
  // Climber subsystem 
  /////////////////////////////////////////////////////////////////////////////
  public static final class CLConsts
  {
    // Global settings

    // Current limit settings - Winch

    // Manual config parameters
    public enum ClimberMode
    {
    }

    // Motion Magic config parameters
  }

  /////////////////////////////////////////////////////////////////////////////
  // Vision (Limelight settings)
  /////////////////////////////////////////////////////////////////////////////
  public static final class VIConsts
  {
    // Limelight-defined streaming states
    public static final int STANDARD      = 0;  // Both cameras side-by-side
    public static final int PIP_MAIN      = 1;  // Limelight with second camera inset
    public static final int PIP_SECONDARY = 2;  // Second camera with limelight inset

    // Limelight-defined LED mode states
    public static final int LED_OFF       = 1;
    public static final int LED_ON        = 3;

    public enum VIRequests
    {
      VISION_OFF,   // Disable limelight LED and enable secondary camera mode
      VISION_ON,    // Enable limelight LED and disable secondary camera mode
      VISION_TOGGLE // Toggle mode
    }

    public static final List<Pose2d> kAprilTagPoses = Collections.unmodifiableList(List.of( //
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0)),        // AprilTag ID: 0 (invalid)
        new Pose2d(new Translation2d(15.079472, 0.245872), new Rotation2d(Units.degreesToRadians(120))),    // AprilTag ID: 1 
        new Pose2d(new Translation2d(16.185134, 0.883666), new Rotation2d(Units.degreesToRadians(120))),    // AprilTag ID: 2 
        new Pose2d(new Translation2d(16.579342, 4.982718), new Rotation2d(Units.degreesToRadians(180))),    // AprilTag ID: 3 
        new Pose2d(new Translation2d(16.579342, 5.547868), new Rotation2d(Units.degreesToRadians(180))),    // AprilTag ID: 4 
        new Pose2d(new Translation2d(14.700758, 8.2042), new Rotation2d(Units.degreesToRadians(270))),      // AprilTag ID: 5 
        new Pose2d(new Translation2d(1.8415, 8.20426), new Rotation2d(Units.degreesToRadians(270))),        // AprilTag ID: 6 
        new Pose2d(new Translation2d(-0.0381, 5.547868), new Rotation2d(Units.degreesToRadians(0))),          // AprilTag ID: 7
        new Pose2d(new Translation2d(-0.0381, 4.982718), new Rotation2d(Units.degreesToRadians(0))),          // AprilTag ID: 8
        new Pose2d(new Translation2d(0.356108, 0.883666), new Rotation2d(Units.degreesToRadians(60))),      // AprilTag ID: 9
        new Pose2d(new Translation2d(1.461516, 0.245872), new Rotation2d(Units.degreesToRadians(60))),      // AprilTag ID: 10
        new Pose2d(new Translation2d(11.904726, 3.713226), new Rotation2d(Units.degreesToRadians(300))),    // AprilTag ID: 11
        new Pose2d(new Translation2d(11.904726, 4.49834), new Rotation2d(Units.degreesToRadians(60))),      // AprilTag ID: 12
        new Pose2d(new Translation2d(11.220196, 4.105148), new Rotation2d(Units.degreesToRadians(180))),    // AprilTag ID: 13
        new Pose2d(new Translation2d(5.320792, 4.105148), new Rotation2d(Units.degreesToRadians(0))),       // AprilTag ID: 14
        new Pose2d(new Translation2d(4.641342, 4.49834), new Rotation2d(Units.degreesToRadians(120))),      // AprilTag ID: 15
        new Pose2d(new Translation2d(4.641342, 3.713226), new Rotation2d(Units.degreesToRadians(240)))      // AprilTag ID: 16
    ));

    // Direction of goal relative to AprilTag 
    public enum VIGoalDirection
    {
      DIRECTION_LEFT,   // Left
      DIRECTION_MIDDLE, // Middle
      DIRECTION_RIGHT   // Right
    }

    /*
     * (needs to be updated to new robot limelight position)
     * public static final double kRobotCenterToFront = Units.inchesToMeters((28.0 + 6.0) / 2); // Depth
     * from limelight to front robot edge
     */
  }

  /////////////////////////////////////////////////////////////////////////////
  // Limelight driving alignment
  /////////////////////////////////////////////////////////////////////////////
  public static final class LLConsts
  {
    // Default calibration
    public static final double kDistance1        = 48;    // distance from bumper in inches for first reference point
    public static final double kVertOffset1      = 0.42;  // LL y reading in degrees for first reference point
    public static final double kDistance2        = 60;    // distance from bumper in inches for second reference point
    public static final double kVertOffset2      = -4.85; // LL y reading in degrees for second reference point

    // Limelight PID driving controls
    public static final double kTurnConstant     = 0.0;
    public static final double kTurnPidKp        = 0.005;
    public static final double kTurnPidKi        = 0.0;
    public static final double kTurnPidKd        = 0.0;
    public static final double kTurnMax          = 0.4;
    public static final double kThrottlePidKp    = 0.011;
    public static final double kThrottlePidKi    = 0.0;
    public static final double kThrottlePidKd    = 0.0;
    public static final double kThrottleMax      = 0.2;
    public static final double kThrottleShape    = 10.0;

    public static final double kTargetAngle      = 0.0;   // Optimal shooting angle
    public static final double kSetPointDistance = 60.0;  // Optimal shooting distance
    public static final double kAngleThreshold   = 3.5;   // Degrees tolerance around optimal
    public static final double kDistThreshold    = 6.0;   // Inches tolerance around optimal
  }

  /////////////////////////////////////////////////////////////////////////////
  // CANdle
  /////////////////////////////////////////////////////////////////////////////
  public static final class LEDConsts
  {
    public enum LEDColor
    {
      LEDCOLOR_OFF,     // CANdle off
      LEDCOLOR_WHITE,   // CANdle white
      LEDCOLOR_RED,     // CANdle red
      LEDCOLOR_ORANGE,  // CANdle orange
      LEDCOLOR_YELLOW,  // CANdle yellow
      LEDCOLOR_GREEN,   // CANdle green
      LEDCOLOR_BLUE,    // CANdle blue
      LEDCOLOR_PURPLE,  // CANdle purple
      LEDCOLOR_DASH     // CANdle color taken from dashboard
    }
  }

}


package frc.robot;

import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
  // bot serial nums
  public static final String kCompSN               = "032B1F7E";
  public static final String kBetaSN               = "03260A3A";

  // Game controller definitions
  public static final int    kDriverPadPort        = 0;
  public static final int    kOperatorPadPort      = 1;

  public static final double kStickDeadband        = 0.15;
  public static final double kTriggerThreshold     = 0.25;

  // Phoenix firmware versions expected
  public static final int    kPhoenix5MajorVersion = ((22 * 256) + 0);
  public static final int    kPhoenix6MajorVersion = 24;

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

    public static final int    kCANID_ShooterLower   = 23;   // Falcon 500
    public static final int    kCANID_ShooterUpper   = 24;   // Falcon 500
    public static final int    kCANID_ShooterRotary  = 25;   // Falcon 500

    public static final int    kCANID_ClimberL       = 27;   // Falcon 500
    public static final int    kCANID_ClimberR       = 28;   // Falcon 500

    public static final int    kCANID_CANdle         = 0;

    // Digital I/Os
    public static final int    kDIO0_NoteInIntake    = 0;
    public static final int    kDIO1_NoteInFeeder    = 1;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Falcon 500
  /////////////////////////////////////////////////////////////////////////////
  public static final class Falcon500
  {
    public static final int    kMaxRPM     = 6380; // free speed for Falcon 500 motor
    public static final double kEncoderCPR = 2048; // CPR is from Falcon 500 Manual
  }

  /////////////////////////////////////////////////////////////////////////////
  // Swerve drive
  /////////////////////////////////////////////////////////////////////////////
  public static final class SWConsts
  {
    /* Individual module constants */
    public static final boolean gyroInvert  = false; // Always ensure Gyro is CCW+ CW-

    /* Controller Invert */
    public static final boolean invertXAxis = false;
    public static final boolean invertYAxis = false;
    public static final boolean invertRAxis = false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Swerve snap constraints
  /////////////////////////////////////////////////////////////////////////////
  public static final class SnapConsts
  {
    public static final double                       kP                                      = 5.0;
    public static final double                       kI                                      = 0.0;
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

    // Roller intake parameters
    public enum RollerMode
    {
      STOP,    // Stop spinning
      ACQUIRE, // Acquire a game piece
      EXPEL,   // Expel a game piece
      SHOOT,    // put into shooter 
      HANDOFF,  // put into feeder 
      HOLD     // Maintain existing setting

    }

    // Rotary manual move parameters
    public enum RotaryMode
    {
      INIT,    // Initialize intake
      INBOARD, // Intake Rotary moving into the robot
      STOPPED, // Intake Rotary stop and hold position
      OUTBOARD // Intake Rotary moving out of the robot
    }

    // Rotary angles - Motion Magic move parameters
    public static final double kRotaryAngleRetracted = -85.0;    // TODO: Tune me!
    public static final double kRotaryAngleHandoff   = 0.0;      // TODO: Tune me!
    public static final double kRotaryAngleDeployed  = 115.0;    // TODO: Tune me!
    public static final double kRotaryAngleMin       = -88.0;    // TODO: Tune me!
    public static final double kRotaryAngleMax       = 125.0;    // TODO: Tune me!
  }

  /////////////////////////////////////////////////////////////////////////////
  // Feeder subsystem 
  /////////////////////////////////////////////////////////////////////////////
  public static final class FDConsts
  {
    // Global settings

    // Feeder intake parameters
    public enum RollerMode
    {
    }

    // Rotary manual move parameters
    public enum RotaryMode
    {
    }

    // Feeder angles - Motion Magic config parameters
  }

  /////////////////////////////////////////////////////////////////////////////
  // Shooter subsystem 
  /////////////////////////////////////////////////////////////////////////////
  public static final class SHConsts
  {
    // Global settings

    // Manual config parameters
    public enum ShooterMode
    {
      REVERSE,    // Shooter runs in reverse direction to handle jams
      STOP,       // Shooter is stopped
      SCORE,      // Shooter ramped to an initial speed before shooting
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

    // Climber manual move parameters
    public enum ClimberMode
    {
      INIT,   // Initialize climber
      UP,     // Climber move upward
      STOP,   // Climber stop
      DOWN    // Climber move downward
    }

    // Climber lengths - Motion Magic config parameters
    public static final double kLengthClimbed = 2.0;    // By definition - Climber fully climbed
    public static final double kLengthFull    = 17.0;   // From Mech Design height needed to reach max chain
    public static final double kLengthChain   = 8.0;    // From Mech Design height needed to reach hanging chain
    public static final double kLengthMin     = -0.25;  // Climber minimum allowable length (quarter inch less than stowed)
    public static final double kLengthMax     = 18.25;  // Climber maximum allowable length (2" beyond high length)
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

    //Poses for limelight paths
    public static final Pose2d       kSpeakerPose   = new Pose2d(2.63, 4.03, Rotation2d.fromDegrees(0));
    public static final Pose2d       kAmpPose       = new Pose2d(1.93, 7.31, Rotation2d.fromDegrees(-90));
    public static final Pose2d       kStageCenter   = new Pose2d(5.92, 4.13, Rotation2d.fromDegrees(0));
    public static final Pose2d       kStageLeft     = new Pose2d(4.3, 5.0, Rotation2d.fromDegrees(120));
    public static final Pose2d       kStageRight    = new Pose2d(4.3, 3.2, Rotation2d.fromDegrees(-120));

    /////////////////////////////////////////////////////////////////////////////
    // Path on the fly trajectory constraints
    /////////////////////////////////////////////////////////////////////////////
    public static final class PATHConsts
    {
      public static final double kMaxVelocityMps                         = 1.0; // TODO: Slowed from 3.0 for testing
      public static final double kMaxAccelerationMpsSq                   = 1.0; // TODO: Slowed from 3.0 for testing

      public static final double kMaxAngularSpeedRadiansPerSecond        = 1.0 * Math.PI; // TODO: Slowed from 2.0 * Math.PI for testing
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = 1.0 * Math.PI; // TODO: Slowed from 1.5 * Math.PI for testing
    }

    //Path constraints
    public static final PathConstraints kConstraints =
        new PathConstraints(PATHConsts.kMaxVelocityMps, PATHConsts.kMaxAccelerationMpsSq,
            PATHConsts.kMaxAngularSpeedRadiansPerSecond, PATHConsts.kMaxAngularSpeedRadiansPerSecondSquared);
  }

  /////////////////////////////////////////////////////////////////////////////
  // CANdle
  /////////////////////////////////////////////////////////////////////////////
  public static final class LEDConsts
  {
    public enum LEDColor
    {
      OFF,      // CANdle off
      WHITE,    // CANdle white
      RED,      // CANdle red
      ORANGE,   // CANdle orange
      YELLOW,   // CANdle yellow
      GREEN,    // CANdle green
      BLUE,     // CANdle blue
      PURPLE,   // CANdle purple
      DASHBOARD // CANdle color taken from dashboard
    }

    public enum LEDAnimation
    {
      COLORFLOW,    // Single color flow through string
      FIRE,         // Fire pattern from one end of string
      LARSON,       // Ping-pong pattern bouncing between string ends
      RAINBOW,      // Fading rainbow colors
      RGBFADE,      // Fading red, then green, then blue
      SINGLEFADE,   // Fading with a single color
      STROBE,       // Strobe flashing with a single color
      TWINKLE,      // Twinkles leds on
      TWINKLEOFF,   // Twinkles leds off
      CLEARALL,     // Clears animations
      DASHBOARD     // Animation taken from the dashboard
    }
  }

}

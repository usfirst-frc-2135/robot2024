
package frc.robot;

import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
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

  // Timeout constants
  public static final int    kLongCANTimeoutMs     = 100;
  public static final int    kCANTimeoutMs         = 10;

  // PDH constatns
  public static final double kMinCurrent           = 0.125;  // Minimum detectable current from PDH

  // Field constants
  public static final double kGridNodeSpacing      = 0.4667; // 18.375 inches between cube node and peg nodes

  public static final int    kPhoenix5MajorVersion = ((22 * 256) + 0);
  public static final int    kPhoenix6MajorVersion = 23;

  public static final double kAutonomousPeriodSecs = 15.0;

  /////////////////////////////////////////////////////////////////////////////
  // CAN IDs and PWM IDs
  /////////////////////////////////////////////////////////////////////////////
  public static final class Ports
  {
    public static final String kCANCarnivore              = "canivore1";
    public static final String kCANRio                    = "rio";

    // CANivore CAN IDs - Swerve
    public static final int    kCANID_DriveLF             = 1;    //Falcon 500
    public static final int    kCANID_SteerLF             = 2;
    public static final int    kCANID_CANCoderLF          = 3;

    public static final int    kCANID_DriveRF             = 4;    //Falcon 500     
    public static final int    kCANID_SteerRF             = 5;
    public static final int    kCANID_CANCoderRF          = 6;

    public static final int    kCANID_DriveLR             = 7;    //Falcon 500
    public static final int    kCANID_SteerLR             = 8;
    public static final int    kCANID_CANCoderLR          = 9;

    public static final int    kCANID_DriveRR             = 10;   //Falcon 500
    public static final int    kCANID_SteerRR             = 11;
    public static final int    kCANID_CANCoderRR          = 12;

    public static final int    kCANID_Pigeon2             = 13;

    // RoboRIO CAN IDs
    public static final int    kCANID_IntakeRoller        = 15;

    public static final int    kCANID_IntakeRotor         = 16;
    public static final int    kCANID_IntakeRotorCANCoder = 17;

    public static final int    kCANID_FeederRoller        = 18;

    public static final int    kCANID_FeederRotor         = 21;
    public static final int    kCANID_FeederRotorCANCoder = 22;

    public static final int    kCANID_Shooter             = 24;

    public static final int    kCANID_ClimberL            = 25;
    public static final int    kCANID_ClimberR            = 26;

    public static final int    kCANID_CANdle              = 0;

    // Digital I/Os
    // public static final int    kDIO_ExampleDetect = 2;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Falcon 500
  /////////////////////////////////////////////////////////////////////////////
  public static final class Falcon500
  {
    public static int          kMaxRPM     = 6380; // free speed for Falcon 500 motor
    public static final double kEncoderCPR = 2048; // CPR is 2048 from Falcon 500 Manual
  }

  /////////////////////////////////////////////////////////////////////////////
  // Swerve drive
  /////////////////////////////////////////////////////////////////////////////
  public static final class SWConsts
  {
    /* Swerve Constants - 0.427 m (x, y) */
    public static final double                trackWidth                    = Units.inchesToMeters(22.7);
    public static final double                wheelBase                     = Units.inchesToMeters(22.7);

    public static final double                wheelDiameter                 = Units.inchesToMeters(4.0);
    public static final double                wheelCircumference            = wheelDiameter * Math.PI;

    public static final double                driveGearRatio                = 6.75;
    public static final double                steerGearRatio                = 21.43;

    public static final Translation2d[ ]      swerveModuleLocations         =
    {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0), new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    };

    public static final SwerveDriveKinematics swerveKinematics              = new SwerveDriveKinematics(swerveModuleLocations);

    /* Swerve Current Limiting */
    public static final int                   driveSupplyCurrentLimit       = 35;
    public static final int                   driveSupplyCurrentThreshold   = 60;
    public static final double                driveSupplyTimeThreshold      = 0.1;
    public static final boolean               driveSupplyCurrentLimitEnable = true;

    public static final int                   steerSupplyCurrentLimit       = 25;
    public static final int                   steerSupplyCurrentThreshold   = 40;
    public static final double                steerSupplyTimeThreshold      = 0.1;
    public static final boolean               steerSupplyCurrentLimitEnable = true;

    /* Drive Motor PID Values */
    public static final double                driveFFKS                     = 0.0;
    public static final double                driveFFKV                     = 0.0;
    public static final double                driveKP                       = 0.36;
    public static final double                driveKI                       = 0.0;
    public static final double                driveKD                       = 0.0;

    /* Angle Motor PID Values */
    public static final double                steerFFKS                     = 0.0;
    public static final double                steerFFKV                     = 0.0;
    public static final double                steerKP                       = 12.0;
    public static final double                steerKI                       = 0.0;
    public static final double                steerKD                       = 0.0;

    /* Neutral Modes */
    public static final InvertedValue         driveMotorInvert              = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue      driveNeutralMode              = NeutralModeValue.Brake;
    public static final double                driveOpenLoopRamp             = 0.25;
    public static final double                driveClosedLoopRamp           = 0.0;

    public static final InvertedValue         steerMotorInvert              = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue      steerNeutralMode              = NeutralModeValue.Coast;
    public static final double                steerOpenLoopRamp             = 0.0;
    public static final double                steerClosedLoopRamp           = 0.0;

    public static final SensorDirectionValue  steerCanCoderInvert           = SensorDirectionValue.CounterClockwise_Positive;
    public static final boolean               gyroInvert                    = false; // Always ensure Gyro is CCW+ CW-

    /* Drive Motor Characterization Values */
    public static final double                driveKS                       = (0.32 / 12);
    public static final double                driveKV                       = (1.51 / 12);
    public static final double                driveKA                       = (0.27 / 12);

    /* Swerve Profiling Values */
    public static final double                maxSpeed                      = 4.5;  // meters per second
    public static final double                maxAngularVelocity            = 6.0;  // orginially 10.0
    public static final double                maxSpeedSlowMode              = 2.25; // meters per second
    public static final double                maxAngularVelocitySlowMode    = 4.0;  // orginially 5.0

    /* Controller Invert */
    public static final boolean               invertXAxis                   = false;
    public static final boolean               invertYAxis                   = false;
    public static final boolean               invertRAxis                   = false;

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

    // Constants for balance
    public static final double kDriveBalancedAngle  = 5.0;    // Pitch values less than this stop driving
    public static final double kDriveBalanceKp      = -0.025; // Amount of power to apply per degree

    public static final double kElbowDriveSlowAngle = 34.0;   // When arm is out beyond this angle - drive is slowed down
  }

  /////////////////////////////////////////////////////////////////////////////
  // Intake Roller
  /////////////////////////////////////////////////////////////////////////////
  public static final class IntakeRollerConsts
  {
    // Global settings

    // Current limit settings - Intake Roller

    // CANCoder Intake Roller absolute offset

    // Manual mode config parameters
    public enum IntakeRollereMode
    {
    }

    public static final double kManualSpeedVolts = 3.0;            // Motor voltage during manual operation (joystick)

    // Motion Magic config parameters
  }

  /////////////////////////////////////////////////////////////////////////////
  // Intake Rotor
  /////////////////////////////////////////////////////////////////////////////
  public static final class IntakeRotorConsts
  {
    // Global settings

    // Extension lengths increase by 0.95" per 90 degrees of elbow rotation (lengths manually adjusted below)

    // Current limit settings - Intake Rotor

    // Calibration

    // Manual config parameters
    public enum IntakeRotorMode
    {
    }

    // Motion Magic config parameters
  }

  /////////////////////////////////////////////////////////////////////////////
  // Wrist
  /////////////////////////////////////////////////////////////////////////////
  public static final class WRConsts
  {
    // Global settings
    public static final double               kGearRatio                = 106.1; // Gear reduction for wrist
    public static final double               kGripperLengthMeters      = 0.3;   // Sim value: 11.8 in
    public static final double               kGripperMassKg            = 3.0;   // Sim value: 6.6 lbs

    public static final double               kAngleMin                 = -2.0;  // Wrist minimum allowable angle (a few degrees less than stowed)
    public static final double               kAngleStow                = 0.0;   // By definition - wrist is 90 degrees perpendicular to arm
    public static final double               kAngleIdle                = 0.0;   // Slightly off stowed value
    public static final double               kAngleScoreLow            = 25.0;  // From Mech Design (floor, feet art 5" high), empirically checked
    public static final double               kAngleScoreMid            = 20.0;  // From Mech Design (1'10-3/4" deep, 2'10" high peg, 1'11-1/2 high cube), ready to score
    public static final double               kAngleScoreHigh           = 20.0;  // From Mech Design (3'3-3/4" deep, 3'10" high peg, 2'11-1/2 high cube), ready to score
    public static final double               kAngleScoreAuto           = 110.0;  // From Mech Design (3'3-3/4" deep, 3'10" high peg, 2'11-1/2 high cube), ready to score
    public static final double               kAngleSubstation          = 0.0; // From Mech Design (3'1-38" above floor)
    public static final double               kAngleMax                 = 115.0; // Wrist maximum allowable angle (a few degrees more than substation/horizontal)

    public static final InvertedValue        kInvertMotor              = InvertedValue.CounterClockwise_Positive; // Motor direction for positive input
    public static final boolean              kInvertCANCoder           = true;

    // Current limit settings - wrist
    public static final double               kSupplyCurrentLimit       = 10.0;  // Supply current limit (after trigger)
    public static final double               kSupplyTriggerCurrent     = 10.0;  // Supply trigger current that will cause limiting
    public static final double               kSupplyTriggerTime        = 0.001; // Supply time duration of trigger that will causing limiting
    public static final boolean              kSupplyCurrentLimitEnable = true;  // Supply current enable

    public static final double               kStatorCurrentLimit       = 20.0; // Stator current limit (after trigger)
    public static final boolean              kStatorCurrentLimitEnable = true; // Stator current enable

    public static final double               kNeutralDeadband          = 0.001; // Wrist motor output deadband

    // CANCoder wrist absolute offset
    public static final double               kCompOffset               = 0.08325; // CANCoder offset angle for comp bot
    public static final double               kBetaOffset               = 0.000;   // CANCoder offset rotations for beta bot
    public static final SensorDirectionValue kSensorDirection          = SensorDirectionValue.Clockwise_Positive;

    // Manual config parameters
    public enum WristMode
    {
      WRIST_INIT,    // Initialize wrist
      WRIST_DOWN,    // Wrist moving down
      WRIST_STOPPED, // Wrist stop and hold position
      WRIST_UP       // Wrist moving up
    }

    public static final double kManualSpeedVolts = 3.0;            // Motor voltage during manual operation (joystick)
    public static final double kScoreSpeedVolts  = 4.0;            // Motor voltage during scoring slam dunk (was 2.0 * manual speed)
    public static final double kBrakeSpeedVolts  = 0.0;            // Motor voltage braking after slam dunk (was 0.25 percent output)

    // Motion Magic config parameters
    public static final double kMMVelocity       = 79.75;          // 10/7/23 Tuned! Wrist motion magic velocity (75% of max motor RPM)
    public static final double kMMAcceleration   = 472.6;          // 10/7/23 Tuned! Wrist motion magic acceleration (target velocity in 1/2s)
    public static final double kMMSCurveStrength = kMMAcceleration * 4.0; // Elbow motion magic jerk limit (1/4 of acceleration time)
    public static final double kS                = 0.0;            // Voltage constant to overcome friction
    public static final double kV                = 0.1129;         // Voltage constant per desired RPM
    public static final double kPidKp            = 1.350;          // Wrist PID proportional constant
    public static final double kPidKi            = 0.0;            // Wrist PID integral constant
    public static final double kPidKd            = 0.0;            // Wrist PID derivative constant

    public static final int    kAllowedError     = 0;              // Wrist PID allowable closed loop error in counts
    public static final double kToleranceDegrees = 2.0;            // Wrist PID tolerance in degrees (1 deg is 0.25" at 15" length)

    public static final double kArbitraryFF      = -0.08; //-0.034;         // Wrist motor output for 90 degrees
    public static final double kMMSafetyTimeout  = 3;              // Seconds allowed for a Motion Magic movement
  }

  /////////////////////////////////////////////////////////////////////////////
  // Grippper
  /////////////////////////////////////////////////////////////////////////////
  public static final class GRConsts
  {
    // Global settings
    public static final boolean kInvertMotor          = false; // Motor direction for positive input

    // Input current limit settings - gripper
    public static final double  kSupplyCurrentLimit   = 30.0;  // Default supply current limit (after trigger)
    public static final double  kSupplyTriggerCurrent = 30.0;  // Trigger current that will cause limiting
    public static final double  kSupplyTriggerTime    = 0.001; // Time duration of trigger that will causing limiting

    public enum GRMode
    {
      GR_STOP,    // stop motor
      GR_ACQUIRE, // acquire game pieces
      GR_EXPEL,   // expel game pieces
      GR_HOLD,    // hold game pieces
    }

    public static final double kGripperSpeedAcquire = 1.0;        // Percent output - Acquire game piece from loading station or floor
    public static final double kGripperSpeedHold    = 3.0 / 12.0; // Percent output - Hold game piece while traversing the field (must be <= 4V equiv)
    public static final double kGripperSpeedExpel   = -0.2;       // Percent output - Score game piece on cone node or cube shelf
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
      VISION_TOGGLE // Toggle modes
    }

    public static final List<Pose2d> kAprilTagPoses = Collections.unmodifiableList(List.of( //
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0)),        // AprilTag ID: 0 (invalid)
        new Pose2d(new Translation2d(15.513558, 1.071626), new Rotation2d(Units.degreesToRadians(180))), // AprilTag ID: 1 
        new Pose2d(new Translation2d(15.513558, 2.748026), new Rotation2d(Units.degreesToRadians(180))), // AprilTag ID: 2 
        new Pose2d(new Translation2d(15.513558, 4.424426), new Rotation2d(Units.degreesToRadians(180))), // AprilTag ID: 3 
        new Pose2d(new Translation2d(16.178784, 6.749796), new Rotation2d(Units.degreesToRadians(180))), // AprilTag ID: 4 
        new Pose2d(new Translation2d(0.36195, 6.749796), new Rotation2d(0)),                               // AprilTag ID: 5 
        new Pose2d(new Translation2d(1.0273, 4.424426), new Rotation2d(0)),                                // AprilTag ID: 6 
        new Pose2d(new Translation2d(1.0273, 2.748026), new Rotation2d(0)),                                // AprilTag ID: 7
        new Pose2d(new Translation2d(1.0273, 1.071626), new Rotation2d(0))                                 // AprilTag ID: 8
    ));

    // Direction of goal relative to AprilTag 
    public enum VIGoalDirection
    {
      DIRECTION_LEFT,   // Left
      DIRECTION_MIDDLE, // Middle
      DIRECTION_RIGHT   // Right
    }

    public static final double kATagDepthInGrid    = Units.inchesToMeters(14.25);    // Depth from front of grid to AprilTag - 1'2-1/4"
    public static final double kRobotCenterToFront = Units.inchesToMeters((28.0 + 6.0) / 2); // Depth from limelight to front robot edge
    public static final double kAdjustPathX        = kATagDepthInGrid + kRobotCenterToFront;
    public static final double kAdjustPathY        = Units.inchesToMeters(18.25 / 2 + 18.5 / 2) + 0.06;     // Addition of 6cm to adjust for empirical error 
    public static final double kAdjustSubPathX     = kRobotCenterToFront + Units.inchesToMeters(30); // Robot stop 30 inches from the substation loading zone
    public static final double kAdjustSubPathY     = Units.inchesToMeters(50.5 / 2);
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

  /////////////////////////////////////////////////////////////////////////////
  // Swerve snap
  /////////////////////////////////////////////////////////////////////////////
  public static final class SnapConstants
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
  // Autonomous
  /////////////////////////////////////////////////////////////////////////////
  public static final class AutoConstants
  {
    enum AutoChooser
    {
      AUTOSTOP,           // AutoStop
      AUTOCOMSHORT,       // AutoDriveOffCommunityShort
      AUTOCOMLONG,        // AutoDriveOffCommunityLong
      AUTOCHARGE,         // AutoEngageChargeStation
      AUTOPRESTOP,        // AutoPreloadAndStop
      AUTOPRECOMSHORT,    // AutoPreloadAndDriveOffCommunityShort
      AUTOPRECOMLONG,     // AutoPreloadAndDriveOffCommunityLong
      AUTOPRECHARGE      // AutoPreloadAndEngageChargeStation
    }

    public static final double                       kMaxSpeedMetersPerSecond                       = 2.2;
    public static final double                       kMaxAccelerationMetersPerSecondSquared         = 2.3;
    public static final double                       kMaxAngularVelocityRadiansPerSecond            = 2 * Math.PI;
    public static final double                       kMaxAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI;

    public static final double                       kSlowSpeedMetersPerSecond                      = 1.7;
    public static final double                       kSlowAccelerationMetersPerSecondSquared        = 2.0;

    public static final double                       kChargeSpeedMetersPerSecond                    = 4.0;
    public static final double                       kChargeAccelerationMetersPerSecondSquared      = 6.0;

    public static final double                       kSlowMaxAngularSpeedRadiansPerSecond           = 0.8 * Math.PI;
    public static final double                       kSlowMaxAngularSpeedRadiansPerSecondSquared    =
        Math.pow(kSlowMaxAngularSpeedRadiansPerSecond, 2);

    public static final double                       kMaxAngularSpeedRadiansPerSecond               = 1.2 * Math.PI;
    public static final double                       kMaxAngularSpeedRadiansPerSecondSquared        =
        Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

    public static final double                       kPXController                                  = 1;
    public static final double                       kPYController                                  = 1;
    public static final double                       kPThetaController                              = 5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints                    =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kSlowThetaControllerConstraints                =
        new TrapezoidProfile.Constraints(kSlowMaxAngularSpeedRadiansPerSecond, kSlowMaxAngularSpeedRadiansPerSecondSquared);

    public static TrajectoryConfig createConfig(double maxSpeed, double maxAccel, double startSpeed, double endSpeed)
    {
      TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
      config.setKinematics(SWConsts.swerveKinematics);
      config.setStartVelocity(startSpeed);
      config.setEndVelocity(endSpeed);
      config.addConstraint(new CentripetalAccelerationConstraint(3.0));

      return config;
    }

    // Trajectory Speed Configs
    public static final TrajectoryConfig defaultSpeedConfig =
        new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(SWConsts.swerveKinematics);

    public static final TrajectoryConfig slowSpeedConfig    =
        new TrajectoryConfig(kSlowSpeedMetersPerSecond, kSlowAccelerationMetersPerSecondSquared)
            .setKinematics(SWConsts.swerveKinematics).setStartVelocity(0).setEndVelocity(0);

    // Path following constraints
    public static final PathConstraints  defaultPathConfig  =
        new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared, kMaxAngularVelocityRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);

    public static final PathConstraints  slowPathConfig     =
        new PathConstraints(kSlowSpeedMetersPerSecond, kSlowAccelerationMetersPerSecondSquared,
            kMaxAngularVelocityRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

    public static final PathConstraints  chargePathConfig   =
        new PathConstraints(kChargeSpeedMetersPerSecond, kChargeAccelerationMetersPerSecondSquared,
            kMaxAngularVelocityRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }

}
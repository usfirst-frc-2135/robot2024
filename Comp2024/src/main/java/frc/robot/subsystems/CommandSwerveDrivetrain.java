package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VIConsts;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.util.LimelightHelpers;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem
{
  private final boolean                              m_useLimelight                  = true; // set to false when no limelight to prevent sim errors
  private static final double                        kSimLoopPeriod                  = 0.005; // 5 ms
  private Notifier                                   m_simNotifier                   = null;
  private double                                     m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d                           BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d                           RedAlliancePerspectiveRotation  = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean                                    hasAppliedOperatorPerspective   = false;
  /* Speaker AprilTag Pose for calculating distance */
  private Pose2d                                     m_allianceSpeakerATPose         = new Pose2d( );

  private final SwerveRequest.ApplyChassisSpeeds     AutoRequest                     = new SwerveRequest.ApplyChassisSpeeds( );

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization     =
      new SwerveRequest.SysIdSwerveTranslation( );
  private final SwerveRequest.SysIdSwerveRotation    RotationCharacterization        = new SwerveRequest.SysIdSwerveRotation( );
  private final SwerveRequest.SysIdSwerveSteerGains  SteerCharacterization           = new SwerveRequest.SysIdSwerveSteerGains( );

  /* Use one of these sysidroutines for your particular test */
  private SysIdRoutine                               SysIdRoutineTranslation         = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(4), null, (state) -> SignalLogger.writeString("state", state.toString( ))),
      new SysIdRoutine.Mechanism((volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this));
  private final SysIdRoutine                         SysIdRoutineRotation            = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(4), null, (state) -> SignalLogger.writeString("state", state.toString( ))),
      new SysIdRoutine.Mechanism((volts) -> setControl(RotationCharacterization.withVolts(volts)), null, this));
  private final SysIdRoutine                         SysIdRoutineSteer               = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(7), null, (state) -> SignalLogger.writeString("state", state.toString( ))),
      new SysIdRoutine.Mechanism((volts) -> setControl(SteerCharacterization.withVolts(volts)), null, this));

  /* Change this to the sysid routine you want to test */
  private final SysIdRoutine                         RoutineToApply                  = SysIdRoutineTranslation;

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance                 inst                            = NetworkTableInstance.getDefault( );

  /* Robot pose for field positioning */
  private final NetworkTable                         table                           = inst.getTable("Pose");
  private final DoubleArrayPublisher                 fieldPub                        =
      table.getDoubleArrayTopic("llPose").publish( );
  private final StringPublisher                      fieldTypePub                    = table.getStringTopic(".type").publish( );

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
      SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    configurePathPlanner( );
    if (Utils.isSimulation( ))
    {
      startSimThread( );
    }
  }

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, modules);
    configurePathPlanner( );
    if (Utils.isSimulation( ))
    {
      startSimThread( );
    }
  }

  private void configurePathPlanner( )
  {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations)
    {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm( ));
    }

    AutoBuilder.configureHolonomic(( ) -> this.getState( ).Pose,      // Supplier of current robot pose
        this::seedFieldRelative,                                      // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,                           // Supplier of chassis speeds
        (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),  // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0), new PIDConstants(10, 0, 0), TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius, new ReplanningConfig( )),                // Path following config
        ( ) ->                                                        // Red vs. Blue alliance
        {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance( );
          if (alliance.isPresent( ))
          {
            return alliance.get( ) == DriverStation.Alliance.Red;
          }
          return false;
        },                                                            // Change this if the path needs to be flipped on red vs blue
        this);                                                        // Subsystem for requirements
  }

  public void resetOdometry(Pose2d pose)
  {
    m_odometry.resetPosition(pose.getRotation( ), m_modulePositions, pose);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
  {
    return run(( ) -> this.setControl(requestSupplier.get( )));
  }

  public Command getAutoCommand(String autoName)
  {
    return new PathPlannerAuto(autoName).withName("swervePPAuto");
  }

  public Command getPathCommand(String pathName)
  {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path).withName("swervePPPath");
  }

  /*
   * Both the sysid commands are specific to one particular sysid routine, change
   * which one you're trying to characterize
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
  {
    return RoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction)
  {
    return RoutineToApply.dynamic(direction);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds( )
  {
    return m_kinematics.toChassisSpeeds(getState( ).ModuleStates);
  }

  private void startSimThread( )
  {
    m_lastSimTime = Utils.getCurrentTimeSeconds( );

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(( ) ->
    {
      final double currentTime = Utils.getCurrentTimeSeconds( );
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage( ));
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    /* Periodically try to apply the operator perspective */
    /*
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS
     * state
     */
    /* This allows us to correct the perspective in case the robot code restarts mid-match */
    /* Otherwise, only check and apply the operator perspective if the DS is disabled */
    /*
     * This ensures driving behavior doesn't change until an explicit disable event occurs during
     * testing
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled( ))
    {
      DriverStation.getAlliance( ).ifPresent((allianceColor) ->
      {
        this.setOperatorPerspectiveForward(
            allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation);
        m_allianceSpeakerATPose = VIConsts.kAprilTagPoses.get((allianceColor == Alliance.Red) ? 4 : 7);
        hasAppliedOperatorPerspective = true;
      });
    }

    double speakerTagDistance = this.getState( ).Pose.getTranslation( ).getDistance(m_allianceSpeakerATPose.getTranslation( ));
    SmartDashboard.putNumber("SW_shootDistance", Units.metersToInches(speakerTagDistance));

    if (m_useLimelight && Robot.isReal( ))
    {
      // var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
      // Pose2d llPose = lastResult.getBotPose2d_wpiBlue( );

      LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

      fieldTypePub.set("Field2d");
      fieldPub.set(new double[ ]
      {
          poseEstimate.pose.getX( ), poseEstimate.pose.getY( ), poseEstimate.pose.getRotation( ).getDegrees( )
      });

      if (poseEstimate.tagCount >= 2)
      {
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        m_odometry.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
      }
    }
  }

  public Command drivePathtoPose(CommandSwerveDrivetrain drivetrain, Pose2d pose)
  {
    DataLogManager.log(String.format("drivePathToPose: given alliance %s target pose %s", DriverStation.getAlliance( ), pose));

    return AutoBuilder.pathfindToPoseFlipped(pose, VIConsts.kConstraints, 0.0);
  }
}

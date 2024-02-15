package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SWConsts;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.util.LimelightHelpers;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem
{
  private final boolean                          m_useLimelight = false; // set to false when no limelight to prevent sim errors

  private static final double                    kSimLoopPeriod = 0.005; // 5 ms
  private Notifier                               m_simNotifier  = null;
  private double                                 m_lastSimTime;

  private final SwerveRequest.ApplyChassisSpeeds autoRequest    = new SwerveRequest.ApplyChassisSpeeds( );

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
        (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),  // Consumer of ChassisSpeeds to drive the robot
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

  public Command getAutoPath(String pathName)
  {
    return new PathPlannerAuto(pathName);
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
    if (m_useLimelight)
    {
      var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

      Pose2d llPose = lastResult.getBotPose2d_wpiBlue( );

      if (lastResult.valid)
      {
        addVisionMeasurement(llPose, Timer.getFPGATimestamp( ));
      }
    }
  }

  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter    = new SlewRateLimiter(3);

  public Translation2d driveWithGamepad2(CommandSwerveDrivetrain swerve, XboxController driverPad, boolean fieldRelative)
  {
    // Get x speed. Invert this because Xbox controllers return negative values when pushing forward.
    final var xSpeed =
        -m_xSpeedLimiter.calculate(MathUtil.applyDeadband(driverPad.getLeftY( ), Constants.kStickDeadband)) * SWConsts.maxSpeed;

    // Get y speed or sideways/strafe speed. Invert this because a positive value is needed when
    // pulling left. Xbox controllers return positive values when pulling right by default.
    final var ySpeed =
        -m_ySpeedLimiter.calculate(MathUtil.applyDeadband(driverPad.getLeftX( ), Constants.kStickDeadband)) * SWConsts.maxSpeed;

    // Get rate of angular rotation. Invert this because a positive value is needed when pulling to
    // the left (CCW is positive in mathematics). Xbox controllers return positive values when pulling
    // to the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(driverPad.getRightX( ), 0.02)) * SWConsts.maxAngularVelocity;

    Translation2d swerveTranslation = new Translation2d(xSpeed, ySpeed);

    return swerveTranslation;

  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FDConsts.FDRollerMode;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.RollerMode;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.VIConsts;
import frc.robot.autos.AutoLeave;
import frc.robot.autos.AutoPreloadLeave;
import frc.robot.autos.AutoPreloadScore;
import frc.robot.autos.AutoPreloadSteal;
import frc.robot.autos.AutoScore4;
import frc.robot.autos.AutoTest;
import frc.robot.commands.ActionAcquireNote;
import frc.robot.commands.ActionExpelNote;
import frc.robot.commands.ActionHandoff;
import frc.robot.commands.ActionPrepareToClimb;
import frc.robot.commands.ActionRetractIntake;
import frc.robot.commands.ActionScoreAmp;
import frc.robot.commands.ActionScoreSpeaker;
import frc.robot.commands.LogCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Vision;

/****************************************************************************
 * 
 * This class is where the bulk of the robot is declared. Since Command-based is a "declarative"
 * paradigm, very little robot logic should actually be handled in the Robot periodic methods (other
 * than the scheduler calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer
{
  private final boolean                               m_macOSXSim    = false;

  // Joysticks
  private static final CommandXboxController          m_driverPad    = new CommandXboxController(Constants.kDriverPadPort);
  private static final CommandXboxController          m_operatorPad  = new CommandXboxController(Constants.kOperatorPadPort);

  private double                                      MaxSpeed       = TunerConstants.kSpeedAt12VoltsMps; // Maximum top speed
  private double                                      MaxAngularRate = 3.0 * Math.PI;                     // 1.5 rotations per second max angular velocity
  private Command                                     m_autoCommand;

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric            drive          = new SwerveRequest.FieldCentric( ) //
      .withDeadband(MaxSpeed * Constants.kStickDeadband)                  //
      .withRotationalDeadband(MaxAngularRate * Constants.kStickDeadband)  //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);            // We want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake        brake          = new SwerveRequest.SwerveDriveBrake( );
  private final SwerveRequest.FieldCentricFacingAngle facing         = new SwerveRequest.FieldCentricFacingAngle( )  //
      .withVelocityX(-m_driverPad.getLeftY( ) * MaxSpeed)                 //
      .withVelocityY(-m_driverPad.getLeftX( ) * MaxSpeed);                // Field centric drive facing a direction
  @SuppressWarnings("unused")
  private final SwerveRequest.PointWheelsAt           point          = new SwerveRequest.PointWheelsAt( );
  private final SwerveRequest.RobotCentric            aim            = new SwerveRequest.RobotCentric( );
  private final SwerveRequest.Idle                    idle           = new SwerveRequest.Idle( );

  private final Telemetry                             logger         = new Telemetry(MaxSpeed);

  // The robot's shared subsystems
  private final LED                                   m_led          = new LED( );
  private final Power                                 m_power        = new Power( );
  private final Vision                                m_vision       = new Vision( );

  // These subsystems may use LED or vision and must be created afterward
  private final CommandSwerveDrivetrain               m_drivetrain   = TunerConstants.DriveTrain;
  private final Intake                                m_intake       = new Intake( );
  private final Shooter                               m_shooter      = new Shooter( );
  private final Feeder                                m_feeder       = new Feeder( );
  private final Climber                               m_climber      = new Climber( );

  Command                                             m_autoCmd;

  // Chooser for autonomous commands - all starting from poses 1-3
  enum AutoChooser
  {
    AUTOSTOP,                // AutoStop - sit still, do nothing
    AUTOLEAVE,               // Leave starting zone avoiding spikes
    AUTOPRELOADONLY,         // Score preloaded game piece from starting pose
    AUTOPRELOADANDLEAVE,     // Score preload at waypoints P1-P3 and leave starting zone
    AUTOPRELOADP0LEAVE,      // Score preload at waypoint P0 and leave starting zone
    AUTOPRELOADP4LEAVE,      // Score preload at waypoint P4 and leave starting zone
    AUTOPRELOADSCOREANOTHER, // Score preload at waypoints P1-P3 and score another from nearest spike
    AUTOSCORE4,              // Score preload at waypoints P1-P3 and pike notes at S1-S3
    AUTOTESTPATH             // Run a selected test path
  }

  enum AutoChooser2
  {
    AUTOSTOP,         // AutoStop - sit still, do nothing
    AUTOLEAVE,        // Leave starting zone avoiding spikes
    AUTOPRELOADLEAVE, // Score preload at waypoints P0, P2, and P4 and leave starting zone
    AUTOPRELOADSCORE, // Score preload at waypoints P1-P3 and score another from nearest spike
    AUTOPRELOADSTEAL, // Score preload at waypoints P1-P3 and steal centerline notes
    AUTOSCORE4,       // Score preload at waypoints P1-P3 and spike notes at S1-S3
    AUTOTEST          // Run a selected test auto
  }

  // Chooser for autonomous starting pose
  enum StartPose
  {
    POSE1, // Starting pose 1 - blue left, red right (driver perspective)
    POSE2, // Starting pose 2 - blue/red middle (driver perspective)
    POSE3  // Starting pose 3 - blue right, red left (driver perspective)
  }

  private SendableChooser<AutoChooser> m_autoChooser  = new SendableChooser<>( );
  private SendableChooser<AutoChooser2> m_autoChooser2 = new SendableChooser<>( );
  private SendableChooser<StartPose>   m_startChooser = new SendableChooser<>( );
  private SendableChooser<Integer>     m_odomChooser  = new SendableChooser<>( );

  private final HashMap<String, String> autoMap        = new HashMap<>(Map.ofEntries( //
      Map.entry(AutoChooser2.AUTOSTOP + StartPose.POSE1.toString( ), "Pos1-Stop"),
      Map.entry(AutoChooser2.AUTOSTOP + StartPose.POSE2.toString( ), "Pos2-Stop"),
      Map.entry(AutoChooser2.AUTOSTOP + StartPose.POSE3.toString( ), "Pos3-Stop"),

      Map.entry(AutoChooser2.AUTOLEAVE + StartPose.POSE1.toString( ), "Pos1-Leave1"),
      Map.entry(AutoChooser2.AUTOLEAVE + StartPose.POSE2.toString( ), "Pos2-Leave2"),
      Map.entry(AutoChooser2.AUTOLEAVE + StartPose.POSE3.toString( ), "Pos3-Leave3"),

      Map.entry(AutoChooser2.AUTOPRELOADLEAVE + StartPose.POSE1.toString( ), "Pos1-P0"),
      Map.entry(AutoChooser2.AUTOPRELOADLEAVE + StartPose.POSE2.toString( ), "Pos2-P2"),
      Map.entry(AutoChooser2.AUTOPRELOADLEAVE + StartPose.POSE3.toString( ), "Pos3-P4"),

      Map.entry(AutoChooser2.AUTOPRELOADSCORE + StartPose.POSE1.toString( ), "Pos1-P1_P1-S2_S1-P1"),
      Map.entry(AutoChooser2.AUTOPRELOADSCORE + StartPose.POSE2.toString( ), "Pos2-P2_P2-S2_S2-P2"),
      Map.entry(AutoChooser2.AUTOPRELOADSCORE + StartPose.POSE3.toString( ), "Pos3-P3_P3-S3_S3-P2"),

      Map.entry(AutoChooser2.AUTOPRELOADSTEAL + StartPose.POSE1.toString( ), "Pos1-P0_P0-C1_C1-C2_C2-C3_C3-C4"),
      Map.entry(AutoChooser2.AUTOPRELOADSTEAL + StartPose.POSE2.toString( ), "Pos2-P2_P2-C1_C1-C2_C2-C3_C3-C4"),
      Map.entry(AutoChooser2.AUTOPRELOADSTEAL + StartPose.POSE3.toString( ), "Pos3-P4_C5-C4_C4-C3_C3-C2_C2-C1"),

      Map.entry(AutoChooser2.AUTOSCORE4 + StartPose.POSE1.toString( ), "Pos1-P1_P1-S1_S1-P1_P1-S2_S2-P2_P2-S3_S3-P3"),
      Map.entry(AutoChooser2.AUTOSCORE4 + StartPose.POSE2.toString( ), "Pos2-P2_P2-S2_S2-P2_P2-S1_S1-P1_P1-S3_S3-P3"),
      Map.entry(AutoChooser2.AUTOSCORE4 + StartPose.POSE3.toString( ), "Pos3-P3_P3-S3_S3-P3_P3-S2_S2-P2_P2-S1_S1-P1"),

      Map.entry(AutoChooser2.AUTOTEST + StartPose.POSE1.toString( ), "Pos1-test1"),
      Map.entry(AutoChooser2.AUTOTEST + StartPose.POSE2.toString( ), "Pos2-test2"),
      Map.entry(AutoChooser2.AUTOTEST + StartPose.POSE3.toString( ), "Pos3-test3") //
  ));

  /****************************************************************************
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer( )
  {
    m_drivetrain.getDaqThread( ).setThreadPriority(99);   // Start swerve telemetry thread

    addSmartDashboardWidgets( );                    // Add dashboard widgets for commands

    configureButtonBindings( );                     // Configure game controller buttons

    initDefaultCommands( );                         // Initialize subsystem default commands

    initAutonomousChooser( );                       // Build autonomous chooser and put on dashboard

    initOdometryChooser( );                         // Build odometry chooser and put on dashboard
  }

  /****************************************************************************
   * 
   * Create general dashboard widgets for commands and subsystems
   */
  private void addSmartDashboardWidgets( )
  {
    // For future work to set up Shuffleboard layout from code
    // ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
    // ComplexWidget autoStopEntry = m_autoTab.add("AutoStop", new AutoStop(m_swerve)).withSize(3, 2).withPosition(0, 0);

    // SmartDashboard Buttons
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(m_feeder);
    SmartDashboard.putData(m_climber);

    SmartDashboard.putData("AutoChooserRun", new InstantCommand(( ) -> getAutonomousCommand( )));

    SmartDashboard.putData("LEDRun", m_led.getLEDCommand(LEDColor.DASHBOARD, LEDAnimation.DASHBOARD));

    SmartDashboard.putNumber("SW_SetPoseX", 0.0);
    SmartDashboard.putNumber("SW_SetPoseY", 0.0);
    SmartDashboard.putNumber("SW_SetPoseRot", 0.0);
    SmartDashboard.putData("SwSetOdometry", new InstantCommand(( ) -> setOdometryFromDashboard( )).ignoringDisable(true));

    SmartDashboard.putData("ActionAcquireNote", new ActionAcquireNote(m_intake, m_led));
    SmartDashboard.putData("ActionExpelNote", new ActionExpelNote(m_intake, m_led));
    SmartDashboard.putData("ActionHandoff", new ActionHandoff(m_intake, m_feeder));
    SmartDashboard.putData("ActionPrepareToClimb", new ActionPrepareToClimb(m_climber, m_feeder));
    SmartDashboard.putData("ActionRetractIntake", new ActionRetractIntake(m_intake, m_led));
    SmartDashboard.putData("ActionScoreAmp", new ActionScoreAmp(m_feeder));
    SmartDashboard.putData("ActionScoreSpeaker", new ActionScoreSpeaker(m_shooter, m_intake, m_led));

    SmartDashboard.putData("InRollStop", m_intake.getMoveToPositionCommand(RollerMode.STOP, m_intake::getIntakePosition));
    SmartDashboard.putData("InRollAcquire", m_intake.getMoveToPositionCommand(RollerMode.ACQUIRE, m_intake::getIntakePosition));
    SmartDashboard.putData("InRollExpel", m_intake.getMoveToPositionCommand(RollerMode.EXPEL, m_intake::getIntakePosition));
    SmartDashboard.putData("InRollShoot", m_intake.getMoveToPositionCommand(RollerMode.SHOOT, m_intake::getIntakePosition));
    SmartDashboard.putData("InRollHold", m_intake.getMoveToPositionCommand(RollerMode.HOLD, m_intake::getIntakePosition));

    SmartDashboard.putData("InRotDeploy", m_intake.getMoveToPositionCommand(RollerMode.HOLD, m_intake::getIntakeDeployed));
    SmartDashboard.putData("InRotRetract", m_intake.getMoveToPositionCommand(RollerMode.HOLD, m_intake::getIntakeRetracted));
    SmartDashboard.putData("InRotHandoff", m_intake.getMoveToPositionCommand(RollerMode.HOLD, m_intake::getIntakeHandoff));

    SmartDashboard.putData("ShRunScore", m_shooter.getShooterScoreCommand( ));
    SmartDashboard.putData("ShRunStop", m_shooter.getShooterStopCommand( ));

    SmartDashboard.putData("FdRollStop", m_feeder.getMoveToPositionCommand(FDRollerMode.STOP, m_feeder::getFeederPosition));
    SmartDashboard.putData("FdRollScore", m_feeder.getMoveToPositionCommand(FDRollerMode.SCORE, m_feeder::getFeederPosition));
    SmartDashboard.putData("FdRollHandoff", m_feeder.getMoveToPositionCommand(FDRollerMode.HANDOFF, m_feeder::getFeederPosition));
    SmartDashboard.putData("FdRollHold", m_feeder.getMoveToPositionCommand(FDRollerMode.HOLD, m_feeder::getFeederPosition));

    SmartDashboard.putData("FdRotAmp", m_feeder.getMoveToPositionCommand(FDRollerMode.HOLD, m_feeder::getFeederAmp));
    SmartDashboard.putData("FdRotClimb", m_feeder.getMoveToPositionCommand(FDRollerMode.HOLD, m_feeder::getFeederClimb));
    SmartDashboard.putData("FdRotHandoff", m_feeder.getMoveToPositionCommand(FDRollerMode.HOLD, m_feeder::getFeederHandoff));

    SmartDashboard.putData("ClRunExtended", m_climber.getMoveToPositionCommand(m_climber::getClimberFullyExtended));
    SmartDashboard.putData("ClRunChain", m_climber.getMoveToPositionCommand(m_climber::getClimberChainLevel));
    SmartDashboard.putData("ClRunClimbed", m_climber.getMoveToPositionCommand(m_climber::getClimberClimbed));
    SmartDashboard.putData("CLCalibrate", m_climber.getCalibrateCommand( ));
  }

  /****************************************************************************
   * 
   * Define button-command mappings. Buttons are created by instantiating a GenericHID or one of its
   * subclasses (Joystick or XboxController), and then passing it to a JoystickButton.
   */
  private void configureButtonBindings( )
  {
    ///////////////////////////////////////////////////////
    //
    // Driver Controller Assignments
    //
    // Driver - A, B, X, Y
    //
    m_driverPad.a( ).whileTrue(m_drivetrain.applyRequest(( ) -> aim                 //
        .withVelocityX(-m_vision.limelight_range_proportional(MaxSpeed))            //
        .withVelocityY(0)                                                 //
        .withRotationalRate(m_vision.limelight_aim_proportional(MaxAngularRate))));
    m_driverPad.b( ).onTrue(new LogCommand("driverPad", "B")); // drive to stage right
    m_driverPad.x( ).onTrue(new LogCommand("driverPad", "X")); // drive to stage left
    m_driverPad.y( ).onTrue(new LogCommand("driverPad", "Y")); // drive to stage center

    //
    // Driver - Bumpers, start, back
    //
    m_driverPad.leftBumper( ).whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kAmpPose));  // drive to amp
    m_driverPad.rightBumper( ).onTrue(new ActionAcquireNote(m_intake, m_led));
    m_driverPad.rightBumper( ).onFalse(new ActionRetractIntake(m_intake, m_led));
    m_driverPad.back( ).whileTrue(m_drivetrain.applyRequest(( ) -> brake));                       // aka View
    m_driverPad.start( ).onTrue(m_drivetrain.runOnce(( ) -> m_drivetrain.seedFieldRelative( )));  // aka Menu

    //
    // Driver - POV buttons
    //
    m_driverPad.pov(0)
        .whileTrue(m_drivetrain.applyRequest(( ) -> facing.withTargetDirection(new Rotation2d(Units.degreesToRadians(0.0)))));
    m_driverPad.pov(90)
        .whileTrue(m_drivetrain.applyRequest(( ) -> facing.withTargetDirection(new Rotation2d(Units.degreesToRadians(270.0)))));
    m_driverPad.pov(180)
        .whileTrue(m_drivetrain.applyRequest(( ) -> facing.withTargetDirection(new Rotation2d(Units.degreesToRadians(180.0)))));
    m_driverPad.pov(270)
        .whileTrue(m_drivetrain.applyRequest(( ) -> facing.withTargetDirection(new Rotation2d(Units.degreesToRadians(90.0)))));
    //
    // Driver Left/Right Trigger
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_driverPad.leftTrigger(Constants.kTriggerThreshold)
        .whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kSpeakerPose)); // drive to speaker
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ActionScoreSpeaker(m_shooter, m_intake, m_led));

    m_driverPad.leftStick( ).onTrue(new LogCommand("driverPad", "left stick"));
    m_driverPad.rightStick( ).onTrue(new LogCommand("driverPad", "right stick"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y
    //
    m_operatorPad.a( ).onTrue(m_shooter.getShooterScoreCommand( ));
    m_operatorPad.b( ).onTrue(m_shooter.getShooterStopCommand( ));
    m_operatorPad.x( ).onTrue(new LogCommand("operPad", "X"));
    m_operatorPad.y( ).onTrue(new ActionExpelNote(m_intake, m_led));

    //
    // Operator - Bumpers, start, back
    //
    m_operatorPad.leftBumper( ).onTrue(new ActionHandoff(m_intake, m_feeder));
    m_operatorPad.rightBumper( ).onTrue(new ActionAcquireNote(m_intake, m_led));
    m_operatorPad.rightBumper( ).onFalse(new ActionRetractIntake(m_intake, m_led));
    m_operatorPad.back( ).toggleOnTrue(m_climber.getJoystickCommand(( ) -> getClimberAxis( )));  // aka View
    m_operatorPad.start( ).onTrue(new InstantCommand(m_vision::rotateCameraStreamMode).ignoringDisable(true)); // aka Menu

    //
    // Operator - POV buttons
    //
    m_operatorPad.pov(0).onTrue(new ActionPrepareToClimb(m_climber, m_feeder));
    m_operatorPad.pov(90).onTrue(new LogCommand("operPad", "POV 90"));
    m_operatorPad.pov(180).onTrue(m_climber.getMoveToPositionCommand(m_climber::getClimberClimbed));
    m_operatorPad.pov(270).onTrue(m_climber.getMoveToPositionCommand(m_climber::getClimberChainLevel));

    //
    // Operator Left/Right Trigger
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_operatorPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new ActionScoreAmp(m_feeder));
    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ActionScoreSpeaker(m_shooter, m_intake, m_led));

    m_operatorPad.leftStick( ).toggleOnTrue(m_feeder.getJoystickCommand(( ) -> getFeederAxis( )));
    m_operatorPad.rightStick( ).toggleOnTrue(m_intake.getJoystickCommand(( ) -> getIntakeAxis( )));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    if (!m_macOSXSim)
    {
      m_drivetrain.setDefaultCommand(                                                  // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                       //
              .withVelocityX(-m_driverPad.getLeftY( ) * MaxSpeed)                      // Drive forward with negative Y (forward)
              .withVelocityY(-m_driverPad.getLeftX( ) * MaxSpeed)                      // Drive left with negative X (left)
              .withRotationalRate(-m_driverPad.getRightX( ) * MaxAngularRate)          // Drive counterclockwise with negative X (left)
          )                                                                            //
              .ignoringDisable(true)                               //
              .withName("CommandSwerveDrivetrain"));
    }
    else // When using simulation on MacOS X, XBox controllers need to be re-mapped due to an Apple bug
    {
      m_drivetrain.setDefaultCommand(                                                   // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                        //
              .withVelocityX(-m_driverPad.getLeftY( ) * MaxSpeed)                       // Drive forward with negative Y (forward)
              .withVelocityY(-m_driverPad.getLeftX( ) * MaxSpeed)                       // Drive left with negative X (left)
              .withRotationalRate(-m_driverPad.getLeftTriggerAxis( ) * MaxAngularRate)  // Drive counterclockwise with negative X (left)
          )                                                                             //
              .ignoringDisable(true)                                //
              .withName("CommandSwerveDrivetrain"));
    }

    // if (Utils.isSimulation()) {  // TODO: needed? maybe fixes config check for simulation where CANcode initializes to 90 degrees?
    //   m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }

    m_drivetrain.registerTelemetry(logger::telemeterize);

    // Default command - Motion Magic hold
    m_intake.setDefaultCommand(m_intake.getHoldPositionCommand(RollerMode.HOLD, m_intake::getIntakePosition));
    m_feeder.setDefaultCommand(m_feeder.getHoldPositionCommand(FDRollerMode.HOLD, m_feeder::getFeederPosition));
    m_climber.setDefaultCommand(m_climber.getHoldPositionCommand(m_climber::getClimberPosition));

    //Default command - manual mode
    // m_intake.setDefaultCommand(m_intake.getJoystickCommand(( ) -> getIntakeAxis( )));
    // m_feeder.setDefaultCommand(m_feeder.getJoystickCommand(( ) -> getFeederAxis( )));
    // m_climber.setDefaultCommand(m_climber.getJoystickCommand(( ) -> getClimberAxis( )));
  }

  /****************************************************************************
   * 
   * Set up autonomous chooser
   */
  private void initAutonomousChooser( )
  {
    // Configure autonomous sendable chooser
    m_autoChooser.setDefaultOption("0 - AutoStop", AutoChooser.AUTOSTOP);
    m_autoChooser.addOption("1 - AutoLeave", AutoChooser.AUTOLEAVE);
    m_autoChooser.addOption("2 - AutoPreloadOnly", AutoChooser.AUTOPRELOADONLY);
    m_autoChooser.addOption("3 - AutoPreloadAndLeave", AutoChooser.AUTOPRELOADANDLEAVE);
    m_autoChooser.addOption("4 - AutoPreloadP0AndLeave", AutoChooser.AUTOPRELOADP0LEAVE);
    m_autoChooser.addOption("5 - AutoPreloadP4AndLeave", AutoChooser.AUTOPRELOADP4LEAVE);
    m_autoChooser.addOption("6 - AutoPreloadAndScoreAnother", AutoChooser.AUTOPRELOADSCOREANOTHER);
    m_autoChooser.addOption("7 - AutoScore4", AutoChooser.AUTOSCORE4);
    m_autoChooser.addOption("8 - AutoTestPath", AutoChooser.AUTOTESTPATH);
    SmartDashboard.putData("AutoMode", m_autoChooser);

    // Configure autonomous sendable chooser
    m_autoChooser2.setDefaultOption("0 - AutoStop", AutoChooser2.AUTOSTOP);
    m_autoChooser2.addOption("1 - AutoLeave", AutoChooser2.AUTOLEAVE);
    m_autoChooser2.addOption("2 - AutoPreloadLeave", AutoChooser2.AUTOPRELOADLEAVE);
    m_autoChooser2.addOption("3 - AutoPreloadScore", AutoChooser2.AUTOPRELOADSCORE);
    m_autoChooser2.addOption("4 - AutoPreloadSteal", AutoChooser2.AUTOPRELOADSTEAL);
    m_autoChooser2.addOption("5 - AutoScore4", AutoChooser2.AUTOSCORE4);
    m_autoChooser2.addOption("6 - AutoTestPath", AutoChooser2.AUTOTEST);
    SmartDashboard.putData("AutoMode2", m_autoChooser2);

    // Configure starting pose sendable chooser
    m_startChooser.setDefaultOption("POSE1", StartPose.POSE1);
    m_startChooser.addOption("POSE2", StartPose.POSE2);
    m_startChooser.addOption("POSE3", StartPose.POSE3);
    SmartDashboard.putData("StartPose", m_startChooser);
  }

  /****************************************************************************
   * 
   * Use this to pass the autonomous command to the main Robot class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand( )
  {
    String pathName = null;
    AutoChooser mode = m_autoChooser.getSelected( );
    StartPose startPose = m_startChooser.getSelected( );
    int poseValue = 0;
    int altpos1 = 0;
    int altpos2 = 0;

    if (m_autoCommand != null)
      m_autoCommand.cancel( );

    switch (startPose)
    {
      default :
        DataLogManager.log(String.format("RobotContainer: Invalid auto start pose %s", startPose));

      case POSE1 :
        poseValue = 1;
        altpos1 = 2;
        altpos2 = 3;
        break;

      case POSE2 :
        poseValue = 2;
        altpos1 = 3;
        altpos2 = 1;
        break;

      case POSE3 :
        poseValue = 3;
        altpos1 = 2;
        altpos2 = 1;
        break;
    }

    pathName = "DriveP" + poseValue;
    DataLogManager.log(String.format("getAutonomousCommand: Initial path %s", pathName));

    // The selected command will be run in autonomous
    switch (mode)
    {
      default :
      case AUTOSTOP :
        m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);
        break;

      case AUTOLEAVE :
        m_autoCommand = m_drivetrain.getAutoCommand(poseValue == 2 ? "DriveS2" : "LeaveS" + poseValue);
        break;

      case AUTOPRELOADONLY :
        m_autoCommand = new ActionScoreSpeaker(m_shooter, m_intake, m_led);
        break;

      case AUTOPRELOADANDLEAVE :
        m_autoCommand = new SequentialCommandGroup(
        // @formatter:off
            new LogCommand(mode.toString(), "Drive to scoring pose"),
            m_drivetrain.getAutoCommand(pathName),

            new LogCommand(mode.toString(), "Score preloaded note"),
            new ActionScoreSpeaker(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Leave zone"),
            m_drivetrain.getAutoCommand(poseValue == 2 ? "DriveS2" : "LeaveS" + poseValue));
        // @formatter:on
        break;

      case AUTOPRELOADP0LEAVE :
        m_autoCommand = new SequentialCommandGroup(
        // @formatter:off
            m_drivetrain.getAutoCommand("DriveP0"),
            new ActionScoreSpeaker(m_shooter, m_intake, m_led), 
            m_drivetrain.getAutoCommand("LeaveS1")
        // @formatter:on
        );
        break;

      case AUTOPRELOADP4LEAVE :
        m_autoCommand = new SequentialCommandGroup(
        // @formatter:off
            new WaitCommand(5), m_drivetrain.getAutoCommand("DriveP4"),
            new ActionScoreSpeaker(m_shooter, m_intake, m_led), 
            m_drivetrain.getAutoCommand("LeaveS3")
        // @formatter:on
        );
        break;

      case AUTOPRELOADSCOREANOTHER :
        m_autoCommand = new SequentialCommandGroup(
        // @formatter:off
            new LogCommand(mode.toString(), "Drive to scoring pose"),
            m_drivetrain.getAutoCommand(pathName), 

            new LogCommand(mode.toString(), "Score preloaded note"),
            new ActionScoreSpeaker(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Deploy intake before moving"),
            m_intake.getMoveToPositionCommand(INConsts.RollerMode.ACQUIRE, m_intake::getIntakeDeployed),

            new WaitCommand(0.5), // TODO - do we need this? The intake command will run to completion first

            new LogCommand(mode.toString(), "Drive to spike while intaking"),
            new ParallelCommandGroup(
                m_drivetrain.getAutoCommand("DriveS" + poseValue),
                new ActionAcquireNote(m_intake, m_led).withTimeout(1.5)
            ),
            
            new LogCommand(mode.toString(), "Drive to scoring pose"),
            m_drivetrain.getAutoCommand("ScoreS" + poseValue),

            new LogCommand(mode.toString(), "Score note"),
            new ActionScoreSpeaker(m_shooter, m_intake, m_led),
            
            new LogCommand(mode.toString(), "Turn off intake rollers"), 
            m_intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, m_intake::getIntakePosition),

            m_drivetrain.getAutoCommand("LeaveS" + poseValue)
        // @formatter:on
        );  //
        break;

      case AUTOSCORE4 :
        m_autoCommand = new SequentialCommandGroup(
        // @formatter:off
            new LogCommand(mode.toString(), "Drive to scoring pose"),
            m_drivetrain.getAutoCommand(pathName),

            new LogCommand(mode.toString(), "Score preloaded note"),
            new ActionScoreSpeaker(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Deploy intake before moving"),
            m_intake.getMoveToPositionCommand(INConsts.RollerMode.ACQUIRE, m_intake::getIntakeDeployed),

            new WaitCommand(0.5),  // TODO - do we need this? The intake command will run to completion first

            new LogCommand(mode.toString(), "Drive to spike while intaking"),
            new ParallelCommandGroup( 
                m_drivetrain.getAutoCommand("DriveS" + poseValue),
                new ActionAcquireNote(m_intake, m_led).withTimeout(1.5)
            ),

            new LogCommand(mode.toString(), "Drive to scoring pose"),
            m_drivetrain.getAutoCommand("ScoreS" + poseValue),

            new LogCommand(mode.toString(), "Score note 2"),
            new ActionScoreSpeaker(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Drive to spike while intaking"),
            new ParallelCommandGroup(
                m_drivetrain.getAutoCommand("DriveS" + altpos1),
                new ActionAcquireNote(m_intake, m_led).withTimeout(1.5)
            ),

            new LogCommand(mode.toString(), "Drive to scoring pose"),
            m_drivetrain.getAutoCommand("ScoreS" + altpos1),

            new LogCommand(mode.toString(), "Score note 3"),
            new ActionScoreSpeaker(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Drive to spike while intaking"),
            new ParallelCommandGroup(
                m_drivetrain.getAutoCommand("DriveS" + altpos2),
                new ActionAcquireNote(m_intake, m_led).withTimeout(1.5)
            ), 
            
            new LogCommand(mode.toString(), "Drive to scoring pose"),
            m_drivetrain.getAutoCommand("ScoreS" + altpos2),

            new LogCommand(mode.toString(), "Score note 4"),
            new ActionScoreSpeaker(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Turn off intake rollers"), 
            m_intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, m_intake::getIntakePosition)
        // @formatter:on
        );
        break;
      case AUTOTESTPATH :
        m_autoCommand = m_drivetrain.getAutoCommand("Test");
        break;
    }

    {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      if (DriverStation.getAlliance( ).equals(Optional.of(DriverStation.Alliance.Red)))
        path = path.flipPath( );

      // List<Pose2d> poses = path.getPathPoses( );
      // for (int i = 0; i < poses.size( ); i++)
      //   DataLogManager.log(String.format("Auto path pose: %s", poses.get(i)));

      Pose2d initialPose =
          new PathPlannerTrajectory(path, new ChassisSpeeds( ), new Rotation2d( )).getInitialTargetHolonomicPose( );

      if (initialPose != null)
        m_drivetrain.resetOdometry(new Pose2d(initialPose.getTranslation( ), initialPose.getRotation( )));
    }

    DataLogManager.log(String.format("getAutonomousCommand: Auto mode is %s %s %s", mode, startPose, m_autoCommand.getName( )));

    return m_autoCommand;
  }

  /****************************************************************************
   * 
   * Use this to pass the autonomous command to the main Robot class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand_2( )
  {
    AutoChooser mode = m_autoChooser.getSelected( );
    StartPose startPose = m_startChooser.getSelected( );
    String autoKey = mode.toString( ) + startPose.toString( );

    if (m_autoCommand != null)
    {
      if (m_autoCommand.isScheduled( ))
        m_autoCommand.cancel( );
      m_autoCommand = null;
    }

    String autoName = autoMap.get(autoKey);

    DataLogManager.log(String.format("================================================================="));
    DataLogManager.log(String.format("getAutoCommand: autoKey: %s  autoName: %s", autoKey, autoName));
    DataLogManager.log(String.format("================================================================="));

    if (autoName == null)
      return m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);

    List<PathPlannerPath> ppPathList = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    PathPlannerPath ppPath = ppPathList.get(0);

    DataLogManager.log(String.format("================================================================="));
    DataLogManager.log(String.format("getAutoCommand: ppPath %s", ppPath.toString( )));
    DataLogManager.log(String.format("================================================================="));

    // if (DriverStation.getAlliance( ).equals(Optional.of(DriverStation.Alliance.Red))) // TODO: Is this needed? PP Auto should handle this
    //   ppPath = ppPath.flipPath( );

    Pose2d initialPose =
        new PathPlannerTrajectory(ppPath, new ChassisSpeeds( ), new Rotation2d( )).getInitialTargetHolonomicPose( );

    if (initialPose != null)
      m_drivetrain.resetOdometry(new Pose2d(initialPose.getTranslation( ), initialPose.getRotation( )));

    switch (mode)
    {
      default :
      case AUTOSTOP :
      m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);
        break;
      case AUTOLEAVE :
        m_autoCommand = new AutoLeave(ppPathList, m_drivetrain, m_led);
        break;
      case AUTOPRELOADLEAVE :
        m_autoCommand = new AutoPreloadLeave(ppPathList, m_drivetrain, m_intake, m_shooter, m_led);
        break;
      case AUTOPRELOADSCORE :
        m_autoCommand = new AutoPreloadScore(ppPathList, m_drivetrain, m_intake, m_shooter, m_led);
        break;
      case AUTOPRELOADSTEAL :
        m_autoCommand = new AutoPreloadSteal(ppPathList, m_drivetrain, m_intake, m_shooter, m_led);
        break;
      case AUTOSCORE4 :
        m_autoCommand = new AutoScore4(ppPathList, m_drivetrain, m_intake, m_shooter, m_led);
        break;
      case AUTOTEST :
        m_autoCommand = new AutoTest(ppPathList, m_drivetrain, m_led);
        break;
    }

    DataLogManager.log(String.format("getAutonomousCommand: Auto mode is %s%s startPose %s %s", mode, startPose, startPose,
        m_autoCommand.getName( )));

    return m_autoCommand;
  }

  /****************************************************************************
   * 
   * Set up odometry chooser
   */
  private void initOdometryChooser( )
  {
    // Autonomous Chooser
    m_odomChooser.setDefaultOption("ID1-AprilTag", 1);
    m_odomChooser.addOption("ID2-AprilTag", 2);
    m_odomChooser.addOption("ID3-AprilTag", 3);
    m_odomChooser.addOption("ID4-AprilTag", 4);
    m_odomChooser.addOption("ID5-AprilTag", 5);
    m_odomChooser.addOption("ID6-AprilTag", 6);
    m_odomChooser.addOption("ID7-AprilTag", 7);
    m_odomChooser.addOption("ID8-AprilTag", 8);
    m_odomChooser.addOption("ID9-AprilTag", 9);
    m_odomChooser.addOption("ID10-AprilTag", 10);
    m_odomChooser.addOption("ID11-AprilTag", 11);
    m_odomChooser.addOption("ID12-AprilTag", 12);
    m_odomChooser.addOption("ID13-AprilTag", 13);
    m_odomChooser.addOption("ID14-AprilTag", 14);
    m_odomChooser.addOption("ID15-AprilTag", 15);
    m_odomChooser.addOption("ID16-AprilTag", 16);

    // Configure odometry sendable chooser
    SmartDashboard.putData("AprilTagPose", m_odomChooser);
    SmartDashboard.putData("ResetPose", new InstantCommand(( ) -> setAprilTagOdometry( )).ignoringDisable(true));
  }

  private void setAprilTagOdometry( )
  {
    m_drivetrain.resetOdometry(Constants.VIConsts.kAprilTagPoses.get(m_odomChooser.getSelected( )));
  }

  private void setOdometryFromDashboard( )
  {
    m_drivetrain.resetOdometry(new Pose2d(new Translation2d(            // 
        SmartDashboard.getNumber("SW_SetPoseX", 0.0),  //
        SmartDashboard.getNumber("SW_SetPoseY", 0.0)), //
        Rotation2d.fromDegrees(SmartDashboard.getNumber("SW_SetPoseRot", 0.0)) //
    ));
  }

  /****************************************************************************
   * 
   * Gamepad interfaces
   */
  public CommandXboxController getDriver( )
  {
    return m_driverPad;
  }

  public CommandXboxController getOperator( )
  {
    return m_operatorPad;
  }

  public double getIntakeAxis( )
  {
    return m_operatorPad.getRightX( );
  }

  public double getFeederAxis( )
  {
    return m_operatorPad.getLeftX( );
  }

  public double getClimberAxis( )
  {
    return -m_operatorPad.getRightY( );
  }

  /****************************************************************************
   * 
   * Called by disabledInit - place subsystem initializations here
   */
  public void initialize( )
  {
    m_led.initialize( );
    m_power.initialize( );
    m_vision.initialize( );

    m_intake.initialize( );
    m_shooter.initialize( );
    m_feeder.initialize( );
    m_climber.initialize( );
  }

  /****************************************************************************
   * 
   * Called when user button is pressed - place subsystem fault dumps here
   */
  public void printFaults( )
  {
    m_led.printFaults( );
    m_power.printFaults( );

    m_intake.printFaults( );
    m_shooter.printFaults( );
    m_feeder.printFaults( );
    m_climber.printFaults( );
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void teleopInit( )
  {
    // getAutonomousCommand_2( );

    CommandScheduler.getInstance( ).schedule(m_climber.getCalibrateCommand( ));
  }

}

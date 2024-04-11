// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
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
import frc.robot.Constants.CLConsts;
import frc.robot.Constants.FDConsts;
import frc.robot.Constants.FDConsts.FDRollerMode;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.RollerMode;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.SHConsts.ShooterMode;
import frc.robot.Constants.VIConsts;
import frc.robot.commands.AutoStop;
import frc.robot.commands.ClimberCalibrate;
import frc.robot.commands.ClimberMoveToPosition;
import frc.robot.commands.ClimberMoveWithJoystick;
import frc.robot.commands.Dummy;
import frc.robot.commands.FeederAmpScore;
import frc.robot.commands.FeederHandoff;
import frc.robot.commands.FeederMoveWithJoystick;
import frc.robot.commands.FeederRun;
import frc.robot.commands.IntakeActionAcquire;
import frc.robot.commands.IntakeActionExpel;
import frc.robot.commands.IntakeActionHandoff;
import frc.robot.commands.IntakeActionRetract;
import frc.robot.commands.IntakeActionShoot;
import frc.robot.commands.IntakeMoveWithJoystick;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.LEDSet;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ShooterActionFire;
import frc.robot.commands.ShooterRun;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  private final boolean                               m_macOSXSim    = false;

  // Joysticks
  private static final CommandXboxController          m_driverPad    = new CommandXboxController(Constants.kDriverPadPort);
  private static final CommandXboxController          m_operatorPad  = new CommandXboxController(Constants.kOperatorPadPort);

  private double                                      MaxSpeed       = TunerConstants.kSpeedAt12VoltsMps; // desired top speed
  private double                                      MaxAngularRate = 3.0 * Math.PI;                     // 1.5 rotations per second max angular velocity
  private Command                                     m_autoCommand;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric            drive          = new SwerveRequest.FieldCentric( )
      .withDeadband(MaxSpeed * Constants.kStickDeadband).withRotationalDeadband(MaxAngularRate * Constants.kStickDeadband)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // We want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake        brake          = new SwerveRequest.SwerveDriveBrake( );
  private final SwerveRequest.FieldCentricFacingAngle facing         = new SwerveRequest.FieldCentricFacingAngle( )
      .withVelocityX(-m_driverPad.getLeftY( ) * MaxSpeed).withVelocityY(-m_driverPad.getLeftX( ) * MaxSpeed);
  private final SwerveRequest.PointWheelsAt           point          = new SwerveRequest.PointWheelsAt( );
  private final SwerveRequest.RobotCentric            aim            = new SwerveRequest.RobotCentric( );

  private final Telemetry                             logger         = new Telemetry(MaxSpeed);

  // The robot's shared subsystems
  private final LED                                   m_led          = new LED( );
  private final Power                                 m_power        = new Power( );
  private final Vision                                m_vision       = new Vision( );

  // These subsystems can use LED or vision and must be created afterward
  private final CommandSwerveDrivetrain               m_drivetrain   = TunerConstants.DriveTrain;
  private final Intake                                m_intake       = new Intake( );
  private final Shooter                               m_shooter      = new Shooter( );
  private final Feeder                                m_feeder       = new Feeder( );
  private final Climber                               m_climber      = new Climber( );

  // Chooser for autonomous commands

  Command                                             m_autoCmd;

  enum AutoChooser
  {
    AUTOSTOP,                // AutoStop - do nothing
    AUTOPRELOADONLY,         // Score preloaded game piece
    AUTOLEAVE,               // Leave starting zone
    AUTOPRELOADANDLEAVE,     // Score preload and leave starting zone
    AUTOPRELOADSCOREANOTHER, // Score preload and score another
    AUTOSCORE4,              //
    AUTOP0LEAVE, //
    AUTOP4LEAVE, //
    AUTOTESTPATH             // Run a selected test path
  }

  // Chooser for autonomous starting position

  enum StartPosition
  {
    POSE1, // Starting position 1 - blue left, red right
    POSE2, // Starting position 2 - blue/red middle
    POSE3  // Starting position 3 - blue right, red left
  }

  private static final boolean           m_autoTesting  = true;
  private SendableChooser<AutoChooser>   m_autoChooser  = new SendableChooser<>( );
  private SendableChooser<StartPosition> m_startChooser = new SendableChooser<>( );
  private SendableChooser<Integer>       m_odomChooser  = new SendableChooser<>( );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer( )
  {
    m_drivetrain.getDaqThread( ).setThreadPriority(99);

    addSmartDashboardWidgets( );

    configureButtonBindings( );

    initDefaultCommands( );

    initAutonomousChooser( );

    initOdometryChooser( );
  }

  public double limelight_aim_proportional(CommandSwerveDrivetrain drivetrain)
  {
    double kP = .01;
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= MaxAngularRate;

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  public double limelight_range_proportional(CommandSwerveDrivetrain drivetrain)
  {
    double kP = .06;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;

    // convert to meters per second
    targetingForwardSpeed *= MaxSpeed;

    // invert since ty is positive when the target is above the crosshair
    targetingForwardSpeed *= -1.0;

    return targetingForwardSpeed;
  }

  /****************************************************************************
   * 
   * Create general dashboard widgets for commands and subsystems
   */
  private void addSmartDashboardWidgets( )
  {
    // SmartDashboard Buttons

    // For future work to set up Shuffleboard layout from code
    // ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
    // ComplexWidget autoStopEntry = m_autoTab.add("AutoStop", new AutoStop(m_swerve)).withSize(3, 2).withPosition(0, 0);

    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(m_feeder);
    SmartDashboard.putData(m_climber);

    SmartDashboard.putData("AutoChooserRun", new InstantCommand(( ) -> getAutonomousCommand( )));

    SmartDashboard.putData("LEDRun", new LEDSet(m_led, LEDColor.DASHBOARD, LEDAnimation.DASHBOARD));

    SmartDashboard.putNumber("SW_PoseX", 0.0);
    SmartDashboard.putNumber("SW_PoseY", 0.0);
    SmartDashboard.putNumber("SW_PoseRot", 0.0);
    SmartDashboard.putData("SwSetOdometry", new InstantCommand(( ) -> setOdometryFromPose( )).ignoringDisable(true));

    SmartDashboard.putData("InActionAcquire", new IntakeActionAcquire(m_intake, m_led));
    SmartDashboard.putData("InActionRetract", new IntakeActionRetract(m_intake, m_led));
    SmartDashboard.putData("InActionExpel", new IntakeActionExpel(m_intake, m_led));
    SmartDashboard.putData("InActionShoot", new IntakeActionShoot(m_intake, m_led));
    SmartDashboard.putData("InActionHandoff", new IntakeActionHandoff(m_intake));

    SmartDashboard.putData("InRollStop", new IntakeRun(m_intake, RollerMode.STOP, m_intake::getRotaryPosition));
    SmartDashboard.putData("InRollAcquire", new IntakeRun(m_intake, RollerMode.ACQUIRE, m_intake::getRotaryPosition));
    SmartDashboard.putData("InRollExpel", new IntakeRun(m_intake, RollerMode.EXPEL, m_intake::getRotaryPosition));
    SmartDashboard.putData("InRollShoot", new IntakeRun(m_intake, RollerMode.SHOOT, m_intake::getRotaryPosition));
    SmartDashboard.putData("InRollHandoff", new IntakeRun(m_intake, RollerMode.HANDOFF, m_intake::getRotaryPosition));
    SmartDashboard.putData("InRollHold", new IntakeRun(m_intake, RollerMode.HOLD, m_intake::getRotaryPosition));

    SmartDashboard.putData("InRotDeploy", new IntakeRun(m_intake, RollerMode.HOLD, m_intake::getIntakeDeployed));
    SmartDashboard.putData("InRotRetract", new IntakeRun(m_intake, RollerMode.HOLD, m_intake::getIntakeRetracted));
    SmartDashboard.putData("InRotHandoff", new IntakeRun(m_intake, RollerMode.HOLD, m_intake::getIntakeHandoff));

    SmartDashboard.putData("ShRunScore", new ShooterRun(m_shooter, ShooterMode.SCORE));
    SmartDashboard.putData("ShRunStop", new ShooterRun(m_shooter, ShooterMode.STOP));

    SmartDashboard.putData("FdAmpScore", new FeederAmpScore(m_feeder));
    SmartDashboard.putData("FdHandoff", new FeederHandoff(m_intake, m_feeder));

    SmartDashboard.putData("FdRollStop", new FeederRun(m_feeder, FDConsts.FDRollerMode.STOP, m_feeder::getRotaryPosition));
    SmartDashboard.putData("FdRollScore", new FeederRun(m_feeder, FDConsts.FDRollerMode.SCORE, m_feeder::getRotaryPosition));
    SmartDashboard.putData("FdRollHandoff", new FeederRun(m_feeder, FDConsts.FDRollerMode.HANDOFF, m_feeder::getRotaryPosition));
    SmartDashboard.putData("FdRollHold", new FeederRun(m_feeder, FDConsts.FDRollerMode.HOLD, m_feeder::getRotaryPosition));

    SmartDashboard.putData("FdRotAmp", new FeederRun(m_feeder, FDConsts.FDRollerMode.HOLD, m_feeder::getRotaryAmp));
    SmartDashboard.putData("FdRotClimb", new FeederRun(m_feeder, FDConsts.FDRollerMode.HOLD, m_feeder::getRotaryClimb));
    SmartDashboard.putData("FdRotHandoff", new FeederRun(m_feeder, FDConsts.FDRollerMode.HOLD, m_feeder::getRotaryHandoff));

    SmartDashboard.putData("ClRunExtended", new ClimberMoveToPosition(m_climber, CLConsts.kLengthFull));
    SmartDashboard.putData("ClRunChain", new ClimberMoveToPosition(m_climber, CLConsts.kLengthChain));
    SmartDashboard.putData("ClRunClimbed", new ClimberMoveToPosition(m_climber, CLConsts.kLengthClimbed));
    SmartDashboard.putData("CLCalibrate", new ClimberCalibrate(m_climber));
  }

  /****************************************************************************
   * 
   * Use this method to define your button-command mappings. Buttons can be created by instantiating
   * a GenericHID or one of its subclasses (Joystick or XboxController), and then passing it to a
   * JoystickButton.
   */
  private void configureButtonBindings( )
  {
    ///////////////////////////////////////////////////////
    //
    // Driver Controller Assignments
    //
    // Driver - A, B, X, Y
    m_driverPad.a( ).whileTrue(m_drivetrain.applyRequest(( ) -> aim.withVelocityX(-limelight_range_proportional(m_drivetrain))
        .withVelocityY(0).withRotationalRate(limelight_aim_proportional(m_drivetrain))));
    m_driverPad.b( ).onTrue(new Dummy("driver b"));
    m_driverPad.b( ).onFalse(new InstantCommand(m_vision::setSpeakerId));
    m_driverPad.x( ).onTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kStageLeft));         // drive to stage left
    m_driverPad.y( ).onTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kStageCenter));       // drive to stage center
    //
    // Driver - Bumpers, start, back
    m_driverPad.leftBumper( ).whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kAmpPose));  // drive to amp
    m_driverPad.rightBumper( ).onTrue(new IntakeActionAcquire(m_intake, m_led));
    m_driverPad.rightBumper( ).onFalse(new IntakeActionRetract(m_intake, m_led));
    m_driverPad.back( ).whileTrue(m_drivetrain.applyRequest(( ) -> brake));                       // aka View
    m_driverPad.start( ).onTrue(m_drivetrain.runOnce(( ) -> m_drivetrain.seedFieldRelative( )));  // aka Menu
    //
    // Driver - POV buttons
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
    m_driverPad.leftTrigger(Constants.kTriggerThreshold)
        .whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kSpeakerPose));           // drive to speaker
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ShooterActionFire(m_shooter, m_intake, m_led));

    m_driverPad.leftStick( ).onTrue(new Dummy("driver left stick"));
    m_driverPad.rightStick( ).onTrue(new Dummy("driver right stick"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y
    m_operatorPad.a( ).onTrue(new ShooterRun(m_shooter, ShooterMode.SCORE));
    m_operatorPad.b( ).onTrue(new ShooterRun(m_shooter, ShooterMode.STOP));
    m_operatorPad.x( ).onTrue(new Dummy("oper X"));
    m_operatorPad.y( ).onTrue(new IntakeActionExpel(m_intake, m_led));
    //
    // Operator - Bumpers, start, back
    m_operatorPad.leftBumper( ).onTrue(new FeederHandoff(m_intake, m_feeder));
    m_operatorPad.rightBumper( ).onTrue(new IntakeActionAcquire(m_intake, m_led));
    m_operatorPad.rightBumper( ).onFalse(new IntakeActionRetract(m_intake, m_led));
    m_operatorPad.back( ).toggleOnTrue(new ClimberMoveWithJoystick(m_climber, m_operatorPad.getHID( )));  // aka View
    m_operatorPad.start( ).onTrue(new InstantCommand(m_vision::rotateCameraStreamMode).ignoringDisable(true)); // aka Menu
    //
    // Operator - POV buttons
    m_operatorPad.pov(0).onTrue(new ParallelCommandGroup( //
        new FeederRun(m_feeder, FDConsts.FDRollerMode.STOP, m_feeder::getRotaryAmp),
        new ClimberMoveToPosition(m_climber, CLConsts.kLengthFull)));
    m_operatorPad.pov(90).onTrue(new Dummy("POV button 90"));
    m_operatorPad.pov(180).onTrue(new ClimberMoveToPosition(m_climber, CLConsts.kLengthClimbed));
    m_operatorPad.pov(270).onTrue(new ClimberMoveToPosition(m_climber, CLConsts.kLengthChain));
    //
    // Operator Left/Right Trigger
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    m_operatorPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new FeederAmpScore(m_feeder));
    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ShooterActionFire(m_shooter, m_intake, m_led));

    m_operatorPad.leftStick( ).toggleOnTrue(new FeederMoveWithJoystick(m_feeder, m_operatorPad.getHID( )));
    m_operatorPad.rightStick( ).toggleOnTrue(new IntakeMoveWithJoystick(m_intake, m_operatorPad.getHID( )));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    if (!m_macOSXSim)
      m_drivetrain.setDefaultCommand(                                                  // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                       //
              .withVelocityX(-m_driverPad.getLeftY( ) * MaxSpeed)                      // Drive forward with negative Y (forward)
              .withVelocityY(-m_driverPad.getLeftX( ) * MaxSpeed)                      // Drive left with negative X (left)
              .withRotationalRate(-m_driverPad.getRightX( ) * MaxAngularRate)          // Drive counterclockwise with negative X (left)
          ).ignoringDisable(true)                                  //
              .withName("CommandSwerveDrivetrain"));
    else // When using simulation on MacOS X, XBox controllers need to be re-mapped due to an Apple bug
      m_drivetrain.setDefaultCommand(                                                   // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                        //
              .withVelocityX(-m_driverPad.getLeftY( ) * MaxSpeed)                       // Drive forward with negative Y (forward)
              .withVelocityY(-m_driverPad.getLeftX( ) * MaxSpeed)                       // Drive left with negative X (left)
              .withRotationalRate(-m_driverPad.getLeftTriggerAxis( ) * MaxAngularRate)  // Drive counterclockwise with negative X (left)
          ).ignoringDisable(true)                                   //
              .withName("CommandSwerveDrivetrain"));

    // if (Utils.isSimulation()) {
    //   m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }

    m_drivetrain.registerTelemetry(logger::telemeterize);

    // Default command - Motion Magic hold
    m_intake.setDefaultCommand(new IntakeRun(m_intake, RollerMode.HOLD, m_intake::getRotaryPosition, true));
    m_feeder.setDefaultCommand(new FeederRun(m_feeder, FDRollerMode.HOLD, m_feeder::getRotaryPosition, true));
    m_climber.setDefaultCommand(new ClimberMoveToPosition(m_climber));

    //Default command - manual mode
    // m_intake.setDefaultCommand(new IntakeMoveWithJoysticks(m_intake, m_operatorPad.getHID( )));
    // m_feeder.setDefaultCommand(new FeederMoveWithJoystick(m_feeder, m_operatorPad.getHID( )));
    // m_climber.setDefaultCommand(new ClimberMoveWithJoystick(m_climber, m_operatorPad.getHID( )));
  }

  /****************************************************************************
   * 
   * Set up autonomous chooser
   */
  private void initAutonomousChooser( )
  {

    // Configure autonomous sendable chooser
    m_autoChooser.setDefaultOption("0 - AutoStop", AutoChooser.AUTOSTOP);
    m_autoChooser.addOption("1 - AutoPreloadOnly", AutoChooser.AUTOPRELOADONLY);
    m_autoChooser.addOption("2 - AutoLeave", AutoChooser.AUTOLEAVE);
    m_autoChooser.addOption("3 - AutoPreloadAndLeave", AutoChooser.AUTOPRELOADANDLEAVE);
    m_autoChooser.addOption("4 - AutoPreloadAndScoreAnother", AutoChooser.AUTOPRELOADSCOREANOTHER);
    m_autoChooser.addOption("5 - AutoScore4", AutoChooser.AUTOSCORE4);
    m_autoChooser.addOption("6 - AutoTestPath", AutoChooser.AUTOTESTPATH);
    m_autoChooser.addOption("7 - AutoPreloadP0AndLeave", AutoChooser.AUTOP0LEAVE);
    m_autoChooser.addOption("8 - AutoPreloadP4AndLeave", AutoChooser.AUTOP4LEAVE);
    SmartDashboard.putData("AutoMode", m_autoChooser);

    // Configure starting position sendable chooser
    m_startChooser.setDefaultOption("POSE1", StartPosition.POSE1);
    m_startChooser.addOption("POSE2", StartPosition.POSE2);
    m_startChooser.addOption("POSE3", StartPosition.POSE3);
    SmartDashboard.putData("StartPosition", m_startChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand( )
  {
    String pathName = null;
    AutoChooser mode = m_autoChooser.getSelected( );
    StartPosition startPosition = m_startChooser.getSelected( );
    int positionValue = 0;
    int altpos1 = 0;
    int altpos2 = 0;

    if (m_autoCommand != null)
      m_autoCommand.cancel( );

    switch (startPosition)
    {
      default :
        DataLogManager.log(String.format("RobotContainer: invalid position %s", startPosition));

      case POSE1 :
        positionValue = 1;
        altpos1 = 2;
        altpos2 = 3;
        break;

      case POSE2 :
        positionValue = 2;
        altpos1 = 3;
        altpos2 = 1;
        break;

      case POSE3 :
        positionValue = 3;
        altpos1 = 2;
        altpos2 = 1;
        break;
    }

    pathName = "DriveP" + positionValue;
    DataLogManager.log(String.format("getAutonomousCommand: %s", pathName));

    // The selected command will be run in autonomous
    switch (mode)
    {
      default :
      case AUTOSTOP :
        m_autoCommand = new AutoStop(m_drivetrain);
        break;

      case AUTOPRELOADONLY :
        m_autoCommand = new ShooterActionFire(m_shooter, m_intake, m_led);
        break;

      case AUTOLEAVE :
        m_autoCommand = m_drivetrain.getAutoCommand(positionValue == 2 ? "DriveS2" : "LeaveS" + positionValue);
        break;

      case AUTOPRELOADANDLEAVE :
        m_autoCommand = new SequentialCommandGroup(
        // @formatter:off
            new LogCommand(mode.toString(), "Drive to scoring position"),
            m_drivetrain.getAutoCommand(pathName),

            new LogCommand(mode.toString(), "Score preloaded note"),
            new ShooterActionFire(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Leave zone"),
            m_drivetrain.getAutoCommand(positionValue == 2 ? "DriveS2" : "LeaveS" + positionValue));
        // @formatter:on
        break;

      case AUTOPRELOADSCOREANOTHER :
        m_autoCommand = new SequentialCommandGroup(
        // @formatter:off
            new LogCommand(mode.toString(), "Drive to scoring position"),
            m_drivetrain.getAutoCommand(pathName), 

            new LogCommand(mode.toString(), "Score preloaded note"),
            new ShooterActionFire(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Deploy intake before moving"),
            new IntakeRun(m_intake, INConsts.RollerMode.ACQUIRE, m_intake::getIntakeDeployed),

            new WaitCommand(0.5), // TODO - do we need this? The intake command will run to completion first

            new LogCommand(mode.toString(), "Drive to spike while intaking"),
            new ParallelCommandGroup(
                m_drivetrain.getAutoCommand("DriveS" + positionValue),
                new IntakeActionAcquire(m_intake, m_led).withTimeout(1.5)
            ),
            
            new LogCommand(mode.toString(), "Drive to scoring position"),
            m_drivetrain.getAutoCommand("ScoreS" + positionValue),

            new LogCommand(mode.toString(), "Score note"),
            new ShooterActionFire(m_shooter, m_intake, m_led),
            
            new LogCommand(mode.toString(), "Turn off intake rollers"), 
            new IntakeRun(m_intake, INConsts.RollerMode.STOP, m_intake::getRotaryPosition),

            m_drivetrain.getAutoCommand("LeaveS" + positionValue)
        // @formatter:on
        );  //
        break;

      case AUTOSCORE4 :
        m_autoCommand = new SequentialCommandGroup(
        // @formatter:off
            new LogCommand(mode.toString(), "Drive to scoring position"),
            m_drivetrain.getAutoCommand(pathName),

            new LogCommand(mode.toString(), "Score preloaded note"),
            new ShooterActionFire(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Deploy intake before moving"),
            new IntakeRun(m_intake, INConsts.RollerMode.ACQUIRE, m_intake::getIntakeDeployed),

            new WaitCommand(0.5),  // TODO - do we need this? The intake command will run to completion first

            new LogCommand(mode.toString(), "Drive to spike while intaking"),
            new ParallelCommandGroup( 
                m_drivetrain.getAutoCommand("DriveS" + positionValue),
                new IntakeActionAcquire(m_intake, m_led).withTimeout(1.5)
            ),

            new LogCommand(mode.toString(), "Drive to scoring position"),
            m_drivetrain.getAutoCommand("ScoreS" + positionValue),

            new LogCommand(mode.toString(), "Score note 2"),
            new ShooterActionFire(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Drive to spike while intaking"),
            new ParallelCommandGroup(
                m_drivetrain.getAutoCommand("DriveS" + altpos1),
                new IntakeActionAcquire(m_intake, m_led).withTimeout(1.5)
            ),

            new LogCommand(mode.toString(), "Drive to scoring position"),
            m_drivetrain.getAutoCommand("ScoreS" + altpos1),

            new LogCommand(mode.toString(), "Score note 3"),
            new ShooterActionFire(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Drive to spike while intaking"),
            new ParallelCommandGroup(
                m_drivetrain.getAutoCommand("DriveS" + altpos2),
                new IntakeActionAcquire(m_intake, m_led).withTimeout(1.5)
            ), 
            
            new LogCommand(mode.toString(), "Drive to scoring position"),
            m_drivetrain.getAutoCommand("ScoreS" + altpos2),

            new LogCommand(mode.toString(), "Score note 4"),
            new ShooterActionFire(m_shooter, m_intake, m_led),

            new LogCommand(mode.toString(), "Turn off intake rollers"), 
            new IntakeRun(m_intake, INConsts.RollerMode.STOP, m_intake::getRotaryPosition)
        // @formatter:on
        );
        break;
      case AUTOP0LEAVE :
        m_autoCommand = new SequentialCommandGroup(m_drivetrain.getAutoCommand("DriveP0"),
            new ShooterActionFire(m_shooter, m_intake, m_led), m_drivetrain.getAutoCommand("LeaveS1"));
        break;
      case AUTOP4LEAVE :
        m_autoCommand = new SequentialCommandGroup(new WaitCommand(5), m_drivetrain.getAutoCommand("DriveP4"),
            new ShooterActionFire(m_shooter, m_intake, m_led), m_drivetrain.getAutoCommand("LeaveS3"));
        break;
      case AUTOTESTPATH :
        m_autoCommand = m_drivetrain.getAutoCommand("Test");
        break;
    }

    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    if (DriverStation.getAlliance( ).equals(Optional.of(DriverStation.Alliance.Red)))
      path = path.flipPath( );

    if (m_autoTesting)
    {
      List<Pose2d> poses = path.getPathPoses( );
      for (int i = 0; i < poses.size( ); i++)
        DataLogManager.log(String.format("pose: %s", poses.get(i)));
    }

    Pose2d initialPose =
        new PathPlannerTrajectory(path, new ChassisSpeeds( ), new Rotation2d( )).getInitialTargetHolonomicPose( );
    if (initialPose != null)
      m_drivetrain.resetOdometry(new Pose2d(initialPose.getTranslation( ), initialPose.getRotation( )));

    DataLogManager.log(String.format("getAutonomousCommand: mode is %s %s %s", mode, startPosition, m_autoCommand.getName( )));

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
    SmartDashboard.putData("ResetPose", new InstantCommand(( ) -> setOdometry( )).ignoringDisable(true));
  }

  public void setOdometry( )
  {
    m_drivetrain.resetOdometry(Constants.VIConsts.kAprilTagPoses.get(m_odomChooser.getSelected( )));
  }

  public void setOdometryFromPose( )
  {
    m_drivetrain.resetOdometry(new Pose2d(new Translation2d(                    // 
        SmartDashboard.getNumber("SW_PoseX", 0.0),             //
        SmartDashboard.getNumber("SW_PoseY", 0.0)),            //
        Rotation2d.fromDegrees(SmartDashboard.getNumber("SW_PoseRot", 0.0)) //
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

  // Called by disabledInit - place subsystem initializations here

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

  // Called when user button is pressed - place subsystem fault dumps here

  public void faultDump( )
  {
    m_led.faultDump( );
    m_power.faultDump( );

    m_intake.faultDump( );
    m_shooter.faultDump( );
    m_feeder.faultDump( );
    m_climber.faultDump( );
  }

  public void teleopInit( )
  {
    CommandScheduler.getInstance( ).schedule(new ClimberCalibrate(m_climber));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CLConsts;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.INConsts.RollerMode;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.SHConsts.ShooterMode;
import frc.robot.commands.AutoStop;
import frc.robot.commands.ClimberMoveToPosition;
import frc.robot.commands.Dummy;
import frc.robot.commands.IntakeActionAcquire;
import frc.robot.commands.IntakeActionExpel;
import frc.robot.commands.IntakeActionRetract;
import frc.robot.commands.IntakeActionShoot;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.LEDSet;
import frc.robot.commands.ShooterRun;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  private final boolean                        m_macOSXSim     = false;

  // Joysticks
  private static final CommandXboxController   m_driverPad     = new CommandXboxController(Constants.kDriverPadPort);
  private static final CommandXboxController   m_operatorPad   = new CommandXboxController(Constants.kOperatorPadPort);

  private double                               MaxSpeed        = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double                               MaxAngularRate  = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private Command                              m_autoCommand;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric     drive           =
      new SwerveRequest.FieldCentric( ).withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake           = new SwerveRequest.SwerveDriveBrake( );
  private final SwerveRequest.RobotCentric     forwardStraight =
      new SwerveRequest.RobotCentric( ).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt    point           = new SwerveRequest.PointWheelsAt( );

  private final Telemetry                      logger          = new Telemetry(MaxSpeed);

  // The robot's shared subsystems
  private final LED                            m_led           = new LED( );
  private final Power                          m_power         = new Power( );
  private final Vision                         m_vision        = new Vision( );

  // These subsystems can use LED or vision and must be created afterward
  private final CommandSwerveDrivetrain        m_drivetrain    = TunerConstants.DriveTrain;
  private final Intake                         m_intake        = new Intake( );
  private final Shooter                        m_shooter       = new Shooter( );
  private final Feeder                         m_feeder        = new Feeder( );
  private final Climber                        m_climber       = new Climber( );

  // Chooser for autonomous commands

  enum AutoChooser
  {
    AUTOSTOP,                // AutoStop - do nothing
    AUTOPRELOADONLY,         // Score preloaded game piece
    AUTOLEAVE,               // Leave starting zone
    AUTOPRELOADANDLEAVE,     // Score preload and leave starting zone
    AUTOPRELOADSCOREANOTHER, // Score preload and score another
    AUTOTESTPATH             // Run a selected test path
  }

  private static final boolean         m_autoTesting = false;
  private SendableChooser<AutoChooser> m_autoChooser = new SendableChooser<>( );
  private SendableChooser<Integer>     m_odomChooser = new SendableChooser<>( );

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

    SmartDashboard.putData("AutoChooserRun", new InstantCommand(( ) -> runAutonomousCommand( )));

    SmartDashboard.putData("LEDRun", new LEDSet(m_led, LEDColor.LEDCOLOR_DASH));

    SmartDashboard.putData("InActionAcquire", new IntakeActionAcquire(m_intake));
    SmartDashboard.putData("InActionRetract", new IntakeActionRetract(m_intake));
    SmartDashboard.putData("InActionExpel", new IntakeActionExpel(m_intake));
    SmartDashboard.putData("InActionShoot", new IntakeActionShoot(m_intake));

    SmartDashboard.putData("InRollAcquire", new IntakeRun(m_intake, RollerMode.ACQUIRE));
    SmartDashboard.putData("InRollExpel", new IntakeRun(m_intake, RollerMode.EXPEL));
    SmartDashboard.putData("InRollStop", new IntakeRun(m_intake, RollerMode.STOP));
    SmartDashboard.putData("InRollHold", new IntakeRun(m_intake, RollerMode.HOLD));

    SmartDashboard.putData("InRotDeploy", new IntakeRun(m_intake, RollerMode.HOLD, INConsts.kRotaryAngleDeployed));
    SmartDashboard.putData("InRotRetract", new IntakeRun(m_intake, RollerMode.HOLD, INConsts.kRotaryAngleRetracted));
    SmartDashboard.putData("InRotHandoff", new IntakeRun(m_intake, RollerMode.HOLD, INConsts.kRotaryAngleHandoff));

    SmartDashboard.putData("ShRunScore", new ShooterRun(m_shooter, ShooterMode.SCORE));
    SmartDashboard.putData("ShRunStop", new ShooterRun(m_shooter, ShooterMode.STOP));

    SmartDashboard.putData("ClRunExtended", new ClimberMoveToPosition(m_climber, CLConsts.kLengthFull));
    SmartDashboard.putData("ClRunChain", new ClimberMoveToPosition(m_climber, CLConsts.kLengthChain));
    SmartDashboard.putData("ClRunClimbed", new ClimberMoveToPosition(m_climber, CLConsts.kLengthClimbed));
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
    // m_driverPad.a( ).onTrue(new Dummy("driver A"));                      // TODO: temporarily used for CTRE testing
    // m_driverPad.b( ).onTrue(new Dummy("driver B"));                   // TODO: temporarily used for CTRE testing
    m_driverPad.x( ).onTrue(new Dummy("driver X"));
    // m_driverPad.y( ).onTrue(drivetrain.driveToSpeaker(drivetrain));
    //
    // Driver - Bumpers, start, back
    // m_driverPad.leftBumper( ).onTrue(new Dummy("driver left bumper"));   // TODO: temporarily used for CTRE testing
    m_driverPad.rightBumper( ).onTrue(new Dummy("driver right bumper"));
    m_driverPad.back( ).onTrue(new Dummy("driver back")); // aka View
    m_driverPad.start( ).onTrue(new Dummy("driver start")); // aka Menu
    //
    // Driver - POV buttons
    // m_driverPad.pov(0).onTrue(new Dummy("driver 0"));                    // TODO: temporarily used for CTRE testing
    m_driverPad.pov(90).onTrue(new Dummy("driver 90"));
    // m_driverPad.pov(180).onTrue(new Dummy("driver 180"));                // TODO: temporarily used for CTRE testing
    m_driverPad.pov(270).onTrue(new Dummy("driver 270"));
    //
    // Driver Left/Right Trigger
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    m_driverPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new Dummy("driver left trigger"));
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new Dummy("driver right trigger"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y
    m_operatorPad.a( ).onTrue(new ClimberMoveToPosition(m_climber, CLConsts.kLengthClimbed));
    m_operatorPad.b( ).onTrue(new Dummy("oper B"));
    m_operatorPad.x( ).onTrue(new ClimberMoveToPosition(m_climber, CLConsts.kLengthChain));
    m_operatorPad.y( ).onTrue(new ClimberMoveToPosition(m_climber, CLConsts.kLengthFull));
    //
    // Operator - Bumpers, start, back
    m_operatorPad.rightBumper( ).onTrue(new IntakeActionAcquire(m_intake));
    m_operatorPad.rightBumper( ).onFalse(new IntakeActionRetract(m_intake));
    m_operatorPad.leftBumper( ).onTrue(new IntakeActionExpel(m_intake));
    m_operatorPad.leftBumper( ).onFalse(new ShooterRun(m_shooter, ShooterMode.STOP));

    m_operatorPad.back( ).onTrue(new Dummy("oper back")); // aka View
    m_operatorPad.start( ).onTrue(new Dummy("oper start")); // aka Menu
    //
    // Operator - POV buttons
    m_operatorPad.pov(0).onTrue(new Dummy("oper 0"));
    m_operatorPad.pov(90).onTrue(new Dummy("oper 90"));
    m_operatorPad.pov(180).onTrue(new Dummy("oper 180"));
    m_operatorPad.pov(270).onTrue(new Dummy("oper 270"));
    //
    // Operator Left/Right Trigger
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new IntakeActionShoot(m_intake));
    m_operatorPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new Dummy("oper left trigger"));

    ///////////////////////////////////////////////////////
    //
    // From CTRE SwerveWithPathPlanner template - mainly for testing
    m_driverPad.a( ).whileTrue(m_drivetrain.applyRequest(( ) -> brake));
    m_driverPad.b( ).whileTrue(m_drivetrain
        .applyRequest(( ) -> point.withModuleDirection(new Rotation2d(-m_driverPad.getLeftY( ), -m_driverPad.getLeftX( )))));

    // reset the field-centric heading on left bumper press
    m_driverPad.leftBumper( ).onTrue(m_drivetrain.runOnce(( ) -> m_drivetrain.seedFieldRelative( )));

    m_driverPad.pov(0).whileTrue(m_drivetrain.applyRequest(( ) -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    m_driverPad.pov(180).whileTrue(m_drivetrain.applyRequest(( ) -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    if (!m_macOSXSim)
      m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive.withVelocityX(-m_driverPad.getLeftY( ) * MaxSpeed) // Drive forward with
              // negative Y (forward)
              .withVelocityY(-m_driverPad.getLeftX( ) * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-m_driverPad.getRightX( ) * MaxAngularRate) // Drive counterclockwise with negative X (left)
          ).ignoringDisable(true));
    else // When using simulation on MacOS X, XBox controllers need to be re-mapped due to an Apple bug
      m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive.withVelocityX(-m_driverPad.getLeftY( ) * MaxSpeed) // Drive forward with
              // negative Y (forward)
              .withVelocityY(-m_driverPad.getLeftX( ) * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-m_driverPad.getLeftTriggerAxis( ) * MaxAngularRate) // Drive counterclockwise with negative X (left)
          ).ignoringDisable(true));

    // if (Utils.isSimulation()) {
    //   m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }

    m_drivetrain.registerTelemetry(logger::telemeterize);

    // // Default command - Motion Magic hold
    // m_intake.setDefaultCommand(new IntakeRun(m_intake, RollerMode.HOLD));
    // m_climber.setDefaultCommand(new ClimberMoveToPosition(m_climber));
    // m_feeder.setDefaultCommand(new FeederMoveToPosition(m_feeder));

    // Default command - manual mode
    //m_intake.setDefaultCommand(new IntakeRotaryJoysticks(m_intake, m_operatorPad.getHID( )));
    // m_climber.setDefaultCommand(new ClimberRun(m_climber));
    // m_feeder.setDefaultCommand(new FeederRun(m_feeder));
  }

  /****************************************************************************
   * 
   * Set up autonomous chooser
   */
  private void initAutonomousChooser( )
  {

    // Autonomous Chooser
    m_autoChooser.setDefaultOption("0 - AutoStop", AutoChooser.AUTOSTOP);
    m_autoChooser.addOption("1 - AutoPreloadOnly", AutoChooser.AUTOPRELOADONLY);
    m_autoChooser.addOption("2 - AutoLeave", AutoChooser.AUTOLEAVE);
    m_autoChooser.addOption("3 - AutoPreloadAndLeave", AutoChooser.AUTOPRELOADANDLEAVE);
    m_autoChooser.addOption("4 - AutoPreloadAndScoreAnother", AutoChooser.AUTOPRELOADSCOREANOTHER);
    m_autoChooser.addOption("5 - AutoTestPath", AutoChooser.AUTOTESTPATH);

    // Configure autonomous sendable chooser
    SmartDashboard.putData("Auto Mode", m_autoChooser);
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

    // The selected command will be run in autonomous
    switch (mode)
    {
      default :
      case AUTOSTOP :
      case AUTOPRELOADONLY :
        break;
      case AUTOLEAVE :
        pathName = "DriveS1";
        break;
      case AUTOPRELOADANDLEAVE :
      case AUTOPRELOADSCOREANOTHER :
        break;
      case AUTOTESTPATH :
        pathName = "Test";
        break;
    }

    if (pathName == null)
      m_autoCommand = new AutoStop(m_drivetrain);
    else
      m_autoCommand = m_drivetrain.getAutoPath(pathName);

    if (m_autoTesting && pathName != null)
    {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      if (DriverStation.getAlliance( ).equals(Alliance.Red))
        path.flipPath( );

      Pose2d initial = new PathPlannerTrajectory(path, new ChassisSpeeds( ), new Rotation2d( )).getInitialTargetHolonomicPose( );
      if (initial != null)
        m_drivetrain.resetOdometry(new Pose2d(initial.getTranslation( ), initial.getRotation( )));
    }

    DataLogManager.log(String.format("getAutonomousCommand: mode is %s", mode));

    return m_autoCommand;
  }

  public void runAutonomousCommand( )
  {
    Command autoCmd = getAutonomousCommand( );
    autoCmd.schedule( );
  }

  /****************************************************************************
   * 
   * Set up odometry chooser
   */
  private void initOdometryChooser( )
  {
    // Autonomous Chooser
    m_odomChooser.setDefaultOption("ID1 - AprilTag", 1);
    m_odomChooser.addOption("ID2 - AprilTag", 2);
    m_odomChooser.addOption("ID3 - AprilTag", 3);
    m_odomChooser.addOption("ID4 - AprilTag", 4);
    m_odomChooser.addOption("ID5 - AprilTag", 5);
    m_odomChooser.addOption("ID6 - AprilTag", 6);
    m_odomChooser.addOption("ID7 - AprilTag", 7);
    m_odomChooser.addOption("ID8 - AprilTag", 8);
    m_odomChooser.addOption("ID9 - AprilTag", 9);
    m_odomChooser.addOption("ID10 - AprilTag", 10);
    m_odomChooser.addOption("ID11 - AprilTag", 11);
    m_odomChooser.addOption("ID12 - AprilTag", 12);
    m_odomChooser.addOption("ID13 - AprilTag", 13);
    m_odomChooser.addOption("ID14 - AprilTag", 14);
    m_odomChooser.addOption("ID15 - AprilTag", 15);
    m_odomChooser.addOption("ID16 - AprilTag", 16);

    // Configure odometry sendable chooser
    SmartDashboard.putData("Reset Odometry", m_odomChooser);
  }

  public Integer getOdometryOption( )
  {
    return m_odomChooser.getSelected( );
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

}

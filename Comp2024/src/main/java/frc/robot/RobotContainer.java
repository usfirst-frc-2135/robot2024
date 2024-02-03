// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.commands.Dummy;
import frc.robot.commands.LEDSet;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Power;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  private static RobotContainer m_instance;

  // Joysticks
  private final XboxController  m_driverPad   = new XboxController(Constants.kDriverPadPort);
  private final XboxController  m_operatorPad = new XboxController(Constants.kOperatorPadPort);

  public final LED              m_led         = new LED( );
  public final Power            m_power       = new Power( );

  private Field2d               m_field       = new Field2d( );

  // The robot's shared subsystems

  // These subsystems can use LED or vision and must be created afterward

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

  private SendableChooser<AutoChooser> m_autoChooser = new SendableChooser<>( );
  private Command                      m_autoCommand;
  PathPlannerTrajectory                m_autoTrajectory;

  private SendableChooser<Integer>     m_odomChooser = new SendableChooser<>( );

  // Command Scheduler

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer( )
  {
    addSmartDashboardWidgets( );

    configureButtonBindings( );

    initDefaultCommands( );

    initAutonomousChooser( );

    initOdometryChooser( );
  }

  public static RobotContainer getInstance( )
  {
    if (m_instance == null)
      m_instance = new RobotContainer( );
    return m_instance;
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
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("LEDSet", new LEDSet(m_led, LEDColor.LEDCOLOR_DASH));

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
    final JoystickButton driverA = new JoystickButton(m_driverPad, XboxController.Button.kA.value);
    final JoystickButton driverB = new JoystickButton(m_driverPad, XboxController.Button.kB.value);
    final JoystickButton driverX = new JoystickButton(m_driverPad, XboxController.Button.kX.value);
    final JoystickButton driverY = new JoystickButton(m_driverPad, XboxController.Button.kY.value);
    //
    final JoystickButton driverLeftBumper = new JoystickButton(m_driverPad, XboxController.Button.kLeftBumper.value);
    final JoystickButton driverRightBumper = new JoystickButton(m_driverPad, XboxController.Button.kRightBumper.value);
    final JoystickButton driverBack = new JoystickButton(m_driverPad, XboxController.Button.kBack.value); // aka View
    final JoystickButton driverStart = new JoystickButton(m_driverPad, XboxController.Button.kStart.value); // aka Menu
    //
    final POVButton driverUp = new POVButton(m_driverPad, 0);
    final POVButton driverRight = new POVButton(m_driverPad, 90);
    final POVButton driverDown = new POVButton(m_driverPad, 180);
    final POVButton driverLeft = new POVButton(m_driverPad, 270);
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    final Trigger driverLeftTrigger = new Trigger(( ) -> m_driverPad.getLeftTriggerAxis( ) > Constants.kTriggerThreshold);
    final Trigger driverRightTrigger = new Trigger(( ) -> m_driverPad.getRightTriggerAxis( ) > Constants.kTriggerThreshold);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final Trigger driverLeftTrigger = new Trigger(( )->m_driverPad.getRightX() > Constants.kTriggerThreshold);
    // final Trigger driverRightTrigger = new Trigger(( )->m_driverPad.getRightY() > Constants.kTriggerThreshold);

    // Driver - A, B, X, Y
    driverA.onTrue(new Dummy("driver A"));
    driverB.whileTrue(new Dummy("driver B"));
    driverX.whileTrue(new Dummy("driver X"));
    driverY.whileTrue(new Dummy("driver Y"));
    //
    // Driver - Bumpers, start, back
    driverLeftBumper.onTrue(new Dummy("driver left bumper"));
    driverRightBumper.onTrue(new Dummy("driver right bumper"));
    driverBack.onTrue(new Dummy("driver back")); // aka View
    driverStart.onTrue(new Dummy("driver start")); // aka Menu
    //
    // Driver - POV buttons
    driverUp.onTrue(new Dummy("driver 0"));
    driverRight.onTrue(new Dummy("driver 90"));
    driverDown.onTrue(new Dummy("driver 180"));
    driverLeft.onTrue(new Dummy("driver 270"));
    //
    // Driver Left/Right Trigger
    driverLeftTrigger.onTrue(new Dummy("driver left trigger"));
    driverRightTrigger.onTrue(new Dummy("driver right trigger"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    final JoystickButton operA = new JoystickButton(m_operatorPad, XboxController.Button.kA.value);
    final JoystickButton operB = new JoystickButton(m_operatorPad, XboxController.Button.kB.value);
    final JoystickButton operX = new JoystickButton(m_operatorPad, XboxController.Button.kX.value);
    final JoystickButton operY = new JoystickButton(m_operatorPad, XboxController.Button.kY.value);
    //
    final JoystickButton operLeftBumper = new JoystickButton(m_operatorPad, XboxController.Button.kLeftBumper.value);
    final JoystickButton operRightBumper = new JoystickButton(m_operatorPad, XboxController.Button.kRightBumper.value);
    final JoystickButton operBack = new JoystickButton(m_operatorPad, XboxController.Button.kBack.value); // aka View
    final JoystickButton operStart = new JoystickButton(m_operatorPad, XboxController.Button.kStart.value); // aka Menu
    //
    final POVButton operUp = new POVButton(m_operatorPad, 0);
    final POVButton operRight = new POVButton(m_operatorPad, 90);
    final POVButton operDown = new POVButton(m_operatorPad, 180);
    final POVButton operLeft = new POVButton(m_operatorPad, 270);
    // 
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    final Trigger operLeftTrigger = new Trigger(( ) -> m_operatorPad.getLeftTriggerAxis( ) > Constants.kTriggerThreshold);
    final Trigger operRightTrigger = new Trigger(( ) -> m_operatorPad.getRightTriggerAxis( ) > Constants.kTriggerThreshold);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final Trigger operLeftTrigger = new Trigger(( ) -> m_operatorPad.getRightX( ) > Constants.kTriggerThreshold);
    // final Trigger operRightTrigger = new Trigger(( ) -> m_operatorPad.getRightY( ) > Constants.kTriggerThreshold);

    // Operator - A, B, X, Y
    operA.onTrue(new Dummy("oper A"));
    operB.whileTrue(new Dummy("oper B"));
    operX.whileTrue(new Dummy("oper X"));
    operY.whileTrue(new Dummy("oper Y"));
    //
    // Operator - Bumpers, start, back
    operLeftBumper.onTrue(new Dummy("oper left bumper"));
    operRightBumper.onTrue(new Dummy("oper right bumper"));
    operBack.onTrue(new Dummy("oper back")); // aka View
    operStart.onTrue(new Dummy("oper start")); // aka Menu
    //
    // Operator - POV buttons
    operUp.onTrue(new Dummy("oper 0"));
    operRight.onTrue(new Dummy("oper 90"));
    operDown.onTrue(new Dummy("oper 180"));
    operLeft.onTrue(new Dummy("oper 270"));
    //
    // Operator Left/Right Trigger
    operLeftTrigger.onTrue(new Dummy("oper left trigger"));
    operRightTrigger.onTrue(new Dummy("oper right trigger"));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    // m_swerve.setDefaultCommand(new DriveTeleop(m_swerve, m_driverPad));

    // // Default command - Motion Magic hold
    // m_elbow.setDefaultCommand(new ElbowMoveToPosition(m_elbow));
    // m_extension.setDefaultCommand(new ExtensionMoveToPosition(m_extension));
    // m_wrist.setDefaultCommand(new WristMoveToPosition(m_wrist));

    // Default command - manual mode
    //m_elbow.setDefaultCommand(new ElbowRun(m_elbow, m_operatorPad));
    //m_extension.setDefaultCommand(new ExtensionRun(m_extension, m_operatorPad));
    //m_wrist.setDefaultCommand(new WristRun(m_wrist, m_operatorPad));
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
    Alliance alliance = DriverStation.getAlliance( ).get( );

    // The selected command will be run in autonomous
    switch (mode)
    {
      default :
      case AUTOSTOP :
      case AUTOPRELOADONLY :
        break;
      case AUTOLEAVE :
      case AUTOPRELOADANDLEAVE :
        pathName = (alliance == Alliance.Red) ? "leaveStartingZoneRed" : "leaveStartingZoneBlue";
        break;
      case AUTOPRELOADSCOREANOTHER :
        pathName = (alliance == Alliance.Red) ? "driveToAnotherRed" : "driveToAnotherBlue";
        break;
      case AUTOTESTPATH :
        pathName = (alliance == Alliance.Red) ? "driveTestPathRed" : "driveTestPathBlue";
        break;
    }

    // if (pathName != null)
    //   m_autoTrajectory =
    //       new PathPlannerTrajectory(PathPlannerPath.fromPathFile(pathName), new ChassisSpeeds( ), new Rotation2d(0, 0));

    // switch (mode)
    // {
    //   default :
    //   case AUTOSTOP :
    //     m_autoCommand = new AutoStop(m_swerve);
    //     break;
    //   case AUTOPRELOADONLY :
    //     m_autoCommand = new AutoScorePreload(m_swerve);
    //     break;
    //   case AUTOLEAVE :
    //     m_autoCommand = new AutoLeave(m_swerve, "leaveStartingZone", m_autoTrajectory);
    //     break;
    //   case AUTOPRELOADANDLEAVE :
    //     m_autoCommand = new AutoPreloadAndLeave(m_swerve, "leaveStartingZone", m_autoTrajectory);
    //     break;
    //   case AUTOPRELOADSCOREANOTHER :
    //     m_autoCommand = new AutoPreloadAndScoreAnother(m_swerve, "driveToAnother", m_autoTrajectory);
    //     break;
    //   case AUTOTESTPATH :
    //     m_autoCommand = new AutoTestPath(m_swerve, "AutoTestPath", m_autoTrajectory);
    //     break;
    // }

    DataLogManager.log(String.format("getAutonomousCommand: mode is %s path is %s", mode, pathName));

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

    // Configure odometry sendable chooser
    SmartDashboard.putData("Reset Odometry Mode", m_odomChooser);
  }

  public Integer getOdometryOption( )
  {
    return m_odomChooser.getSelected( );
  }

  public XboxController getDriver( )
  {
    return m_driverPad;
  }

  public XboxController getOperator( )
  {
    return m_operatorPad;
  }

}

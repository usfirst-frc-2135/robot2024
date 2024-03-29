// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoChooser;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.EXConsts;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.VIConsts;
import frc.robot.Constants.VIConsts.VIGoalDirection;
import frc.robot.Constants.WRConsts;
import frc.robot.commands.ArmManualMode;
import frc.robot.commands.ArmSetHeightIdle;
import frc.robot.commands.ArmSetHeightScoreHigh;
import frc.robot.commands.ArmSetHeightScoreLow;
import frc.robot.commands.ArmSetHeightScoreMid;
import frc.robot.commands.ArmSetHeightShelf;
import frc.robot.commands.ArmSetHeightStow;
import frc.robot.commands.AutoDriveBalance;
import frc.robot.commands.AutoDrivePath;
import frc.robot.commands.AutoEngageChargeStation;
import frc.robot.commands.AutoPreloadAndEngageChargeStation;
import frc.robot.commands.AutoPreloadAndLeaveCommunityLong;
import frc.robot.commands.AutoPreloadAndLeaveCommunityShort;
import frc.robot.commands.AutoPreloadAndStop;
import frc.robot.commands.AutoStop;
import frc.robot.commands.CameraDisplaySet;
import frc.robot.commands.DriveLimelightPath;
import frc.robot.commands.DriveResetGyro;
import frc.robot.commands.DriveResetOdometry;
import frc.robot.commands.DriveSnap;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.Dummy;
import frc.robot.commands.ElbowMoveToPosition;
import frc.robot.commands.ElbowSetAngleZero;
import frc.robot.commands.ExtensionCalibrate;
import frc.robot.commands.ExtensionMoveToPosition;
import frc.robot.commands.GripperRun;
import frc.robot.commands.LEDSet;
import frc.robot.commands.WristMoveToPosition;
import frc.robot.commands.WristRunBrake;
import frc.robot.commands.WristRunConstant;
import frc.robot.commands.WristSetAngleZero;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  private static RobotContainer        m_instance;

  // Joysticks
  private final XboxController         m_driverPad          = new XboxController(Constants.kDriverPadPort);
  private final XboxController         m_operatorPad        = new XboxController(Constants.kOperatorPadPort);

  // The robot's shared subsystems
  public final LED                     m_led                = new LED( );
  public final Vision                  m_vision             = new Vision( );

  // These subsystems can use LED or vision and must be created afterward
  public final Elbow                   m_elbow              = new Elbow( );
  public final Extension               m_extension          = new Extension(m_elbow);
  public final Wrist                   m_wrist              = new Wrist(m_elbow);
  public final Gripper                 m_gripper            = new Gripper( );
  public final Power                   m_power              = new Power( );
  public final Swerve                  m_swerve             = new Swerve(m_vision);

  // A chooser for autonomous commands
  private SendableChooser<AutoChooser> m_autoChooser        = new SendableChooser<>( );
  private Command                      m_autoCommand;
  PathPlannerTrajectory                m_autoTrajectory;

  private SendableChooser<Integer>     m_odomChooser        = new SendableChooser<>( );

  // Command Scheduler
  public Command                       m_extensionCalibrate = new ExtensionCalibrate(m_extension);

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

    // Autonomous helper commands
    SmartDashboard.putData("AutoDriveBalance", new AutoDriveBalance(m_swerve));

    // Elbow subsytem tests
    SmartDashboard.putData("ElbowStow", new ElbowMoveToPosition(m_elbow, ELConsts.kAngleStow));
    SmartDashboard.putData("ElbowIdle", new ElbowMoveToPosition(m_elbow, ELConsts.kAngleIdle));
    SmartDashboard.putData("ElbowLow", new ElbowMoveToPosition(m_elbow, ELConsts.kAngleScoreLow));
    SmartDashboard.putData("ElbowMid", new ElbowMoveToPosition(m_elbow, ELConsts.kAngleScoreMid));
    SmartDashboard.putData("ElbowHigh", new ElbowMoveToPosition(m_elbow, ELConsts.kAngleScoreHigh));
    SmartDashboard.putData("ElbowShelf", new ElbowMoveToPosition(m_elbow, ELConsts.kAngleSubstation));
    SmartDashboard.putData("ElbowSetAngleToZero", new ElbowSetAngleZero(m_elbow));

    // Extension subsytem tests
    SmartDashboard.putData("ExtensionStow", new ExtensionMoveToPosition(m_extension, m_elbow, EXConsts.kLengthStow));
    SmartDashboard.putData("ExtensionIdle", new ExtensionMoveToPosition(m_extension, m_elbow, EXConsts.kLengthIdle));
    SmartDashboard.putData("ExtensionLow", new ExtensionMoveToPosition(m_extension, m_elbow, EXConsts.kLengthScoreLow));
    SmartDashboard.putData("ExtensionMid", new ExtensionMoveToPosition(m_extension, m_elbow, EXConsts.kLengthScoreMid));
    SmartDashboard.putData("ExtensionHigh", new ExtensionMoveToPosition(m_extension, m_elbow, EXConsts.kLengthScoreHigh));
    SmartDashboard.putData("ExtensionShelf", new ExtensionMoveToPosition(m_extension, m_elbow, EXConsts.kLengthSubstation));
    SmartDashboard.putData("ExtensionCalibrate", new ExtensionCalibrate(m_extension));

    // Wrist subsytem tests
    SmartDashboard.putData("WristStow", new WristMoveToPosition(m_wrist, WRConsts.kAngleStow));
    SmartDashboard.putData("WristIdle", new WristMoveToPosition(m_wrist, WRConsts.kAngleIdle));
    SmartDashboard.putData("WristLow", new WristMoveToPosition(m_wrist, WRConsts.kAngleScoreLow));
    SmartDashboard.putData("WristMid", new WristMoveToPosition(m_wrist, WRConsts.kAngleScoreMid));
    SmartDashboard.putData("WristHigh", new WristMoveToPosition(m_wrist, WRConsts.kAngleScoreHigh));
    SmartDashboard.putData("WristShelf", new WristMoveToPosition(m_wrist, WRConsts.kAngleSubstation));
    SmartDashboard.putData("WristSetAngleToZero", new WristSetAngleZero(m_wrist));
    SmartDashboard.putData("WristRunConstant", new WristRunConstant(m_wrist, true));
    SmartDashboard.putData("WristRunBrake", new WristRunBrake(m_wrist, true));

    // Gripper subsystem test
    SmartDashboard.putData("GripperStop", new GripperRun(m_gripper, GRMode.GR_STOP));
    SmartDashboard.putData("GripperAcquire", new GripperRun(m_gripper, GRMode.GR_ACQUIRE));
    SmartDashboard.putData("GripperHold", new GripperRun(m_gripper, GRMode.GR_HOLD));
    SmartDashboard.putData("GripperExpel", new GripperRun(m_gripper, GRMode.GR_EXPEL));

    // Arm group command tests
    SmartDashboard.putData("ArmSetHeightStow", new ArmSetHeightStow(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightIdle", new ArmSetHeightIdle(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightScoreLow", new ArmSetHeightScoreLow(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightScoreMid", new ArmSetHeightScoreMid(m_elbow, m_extension, m_wrist));
    SmartDashboard.putData("ArmSetHeightScoreHigh", new ArmSetHeightScoreHigh(m_elbow, m_extension, m_wrist));

    // On-the-fly path generation helper tests
    SmartDashboard.putData("DriveResetOdometry", new DriveResetOdometry(m_swerve, m_vision));
    SmartDashboard.putData("DriveLLLeft", new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_LEFT));
    SmartDashboard.putData("DriveLLMiddle", new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_MIDDLE));
    SmartDashboard.putData("DriveLLRight", new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_RIGHT));

    // LED (CANdle) test
    SmartDashboard.putData("LEDSet", new LEDSet(m_led, LEDColor.LEDCOLOR_DASH));

    SmartDashboard.putData("Dummy", new Dummy("2135 is here!")); // Used to test smartdashboard operation
  }

  /****************************************************************************
   * 
   * Use this method to define your button->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
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
    // @formatter:off
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    final Trigger driverLeftTrigger = new Trigger(( )->m_driverPad.getLeftTriggerAxis() > Constants.kTriggerThreshold);
    final Trigger driverRightTrigger = new Trigger(( )->m_driverPad.getRightTriggerAxis() > Constants.kTriggerThreshold);
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    // final Trigger driverLeftTrigger = new Trigger(( )->m_driverPad.getRightX() > Constants.kTriggerThreshold);
    // final Trigger driverRightTrigger = new Trigger(( )->m_driverPad.getRightY() > Constants.kTriggerThreshold);
    // @formatter:on

    // Driver - A, B, X, Y
    driverA.onTrue(new ArmSetHeightIdle(m_elbow, m_extension, m_wrist));
    driverB.whileTrue(new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_RIGHT));
    driverX.whileTrue(new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_LEFT));
    driverY.whileTrue(new DriveLimelightPath(m_swerve, m_vision, VIGoalDirection.DIRECTION_MIDDLE));
    //
    // Driver - Bumpers, start, back
    driverLeftBumper.onTrue(new Dummy("left bumper"));
    driverRightBumper.onTrue(new GripperRun(m_gripper, GRMode.GR_ACQUIRE));
    driverRightBumper.onFalse(new GripperRun(m_gripper, GRMode.GR_HOLD));
    driverBack.onTrue(new DriveResetGyro(m_swerve, driverStart, driverBack)); // aka View
    driverStart.onTrue(new DriveResetGyro(m_swerve, driverStart, driverBack)); // aka Menu
    //
    // Driver - POV buttons
    driverUp.onTrue(new DriveSnap(m_swerve, 0));
    driverRight.onTrue(new DriveSnap(m_swerve, -90));
    driverDown.onTrue(new DriveSnap(m_swerve, 180));
    driverLeft.onTrue(new DriveSnap(m_swerve, 90));
    //
    // Operator Left/Right Trigger
    driverLeftTrigger.onTrue(new Dummy("left trigger"));
    driverRightTrigger.onTrue(new GripperRun(m_gripper, GRMode.GR_EXPEL));
    driverRightTrigger.onFalse(new GripperRun(m_gripper, GRMode.GR_STOP));

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
    operA.toggleOnTrue(new ArmSetHeightScoreLow(m_elbow, m_extension, m_wrist));
    operB.toggleOnTrue(new ArmSetHeightScoreMid(m_elbow, m_extension, m_wrist));
    operX.onTrue(new ArmSetHeightIdle(m_elbow, m_extension, m_wrist));
    operY.onTrue(new ArmSetHeightScoreHigh(m_elbow, m_extension, m_wrist));
    //
    // Operator - Bumpers, start, back
    operLeftBumper.whileTrue(new WristRunConstant(m_wrist, true));
    operLeftBumper.onFalse(new WristRunBrake(m_wrist, true));
    operRightBumper.onTrue(new GripperRun(m_gripper, GRMode.GR_ACQUIRE));
    operRightBumper.onFalse(new GripperRun(m_gripper, GRMode.GR_HOLD));
    operBack.toggleOnTrue(new ArmManualMode(m_elbow, m_extension, m_wrist, m_operatorPad)); // aka View

    operStart.onTrue(new CameraDisplaySet(m_vision, VIConsts.PIP_SECONDARY)); // aka Menu
    //
    // Operator - POV buttons
    operUp.onTrue(new ArmSetHeightShelf(m_elbow, m_extension, m_wrist));
    operRight.onTrue(new Dummy("POV right"));
    operDown.onTrue(new Dummy("POV down"));
    operLeft.onTrue(new Dummy("POV left"));
    //
    // Operator Left/Right Trigger
    operLeftTrigger.whileTrue(new WristRunConstant(m_wrist, false));
    operLeftTrigger.onFalse(new WristRunBrake(m_wrist, false));
    operRightTrigger.onTrue(new GripperRun(m_gripper, GRMode.GR_EXPEL));
    operRightTrigger.onFalse(new GripperRun(m_gripper, GRMode.GR_STOP));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    m_swerve.setDefaultCommand(new DriveTeleop(m_swerve, m_elbow, m_driverPad));

    // Default command - Motion Magic hold
    m_elbow.setDefaultCommand(new ElbowMoveToPosition(m_elbow));
    m_extension.setDefaultCommand(new ExtensionMoveToPosition(m_extension, m_elbow));
    m_wrist.setDefaultCommand(new WristMoveToPosition(m_wrist));

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
    m_autoChooser.addOption("1 - AutoDriveOffCommunityShort", AutoChooser.AUTOCOMSHORT);
    m_autoChooser.addOption("2 - AutoDriveOffCommunityLong", AutoChooser.AUTOCOMLONG);
    m_autoChooser.addOption("3 - AutoEngageChargeStation", AutoChooser.AUTOCHARGE);
    m_autoChooser.addOption("4 - AutoPreloadAndStop", AutoChooser.AUTOPRESTOP);
    m_autoChooser.addOption("5 - AutoPreloadAndLeaveCommunityShort", AutoChooser.AUTOPRECOMSHORT);
    m_autoChooser.addOption("6 - AutoPreloadAndLeaveCommunityLong", AutoChooser.AUTOPRECOMLONG);
    m_autoChooser.addOption("7 - AutoPreloadAndEngageChargeStation", AutoChooser.AUTOPRECHARGE);
    m_autoChooser.addOption("8 - AutoTestPath", AutoChooser.AUTOTESTPATH);
    m_autoChooser.addOption("9 - AutoShortNote", AutoChooser.AUTOSHORTNOTE);

    //m_chooser.addOption("8 - AutoPreloadAndScoreAnother", new AutoPreloadAndScoreAnother(m_swerve));

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
      case AUTOPRESTOP :
        break;
      case AUTOCOMSHORT :
      case AUTOPRECOMSHORT :
        pathName = (alliance == Alliance.Red) ? "driveOutOfCommunityShortRed" : "driveOutOfCommunityShortBlue";
        break;
      case AUTOCOMLONG :
      case AUTOPRECOMLONG :
        pathName = (alliance == Alliance.Red) ? "driveOutOfCommunityLongRed" : "driveOutOfCommunityLongBlue";
        break;
      case AUTOCHARGE :
      case AUTOPRECHARGE :
        pathName = (alliance == Alliance.Red) ? "driveOntoChargeStationRed" : "driveOntoChargeStationBlue";
        break;
      case AUTOTESTPATH :
        pathName = "DriveExample";
        break;
      case AUTOSHORTNOTE :
        pathName = (alliance == Alliance.Red) ? "R_DriveShortNote" : "B_DriveShortNote";
        break;
    }

    if (pathName != null)
      m_autoTrajectory =
          new PathPlannerTrajectory(PathPlannerPath.fromPathFile(pathName), new ChassisSpeeds( ), new Rotation2d(0, 0));

    switch (mode)
    {
      default :
      case AUTOSTOP :
        m_autoCommand = new AutoStop(m_swerve);
        break;
      case AUTOCOMSHORT :
        m_autoCommand = new AutoDrivePath(m_swerve, "driveOutOfCommunityShort", m_autoTrajectory, true);
        break;
      case AUTOCOMLONG :
        m_autoCommand = new AutoDrivePath(m_swerve, "driveOutOfCommunityLong", m_autoTrajectory, true);
        break;
      case AUTOCHARGE :
        m_autoCommand = new AutoEngageChargeStation(m_swerve, "driveOntoChargeStation", m_autoTrajectory);
        break;
      case AUTOPRESTOP :
        m_autoCommand = new AutoPreloadAndStop(m_swerve, m_elbow, m_extension, m_wrist, m_gripper);
        break;
      case AUTOPRECOMSHORT :
        m_autoCommand = new AutoPreloadAndLeaveCommunityShort(m_swerve, m_elbow, m_extension, m_wrist, m_gripper,
            "AutoPreloadAndLeaveCommunityShort", m_autoTrajectory);
        break;
      case AUTOPRECOMLONG :
        m_autoCommand = new AutoPreloadAndLeaveCommunityLong(m_swerve, m_elbow, m_extension, m_wrist, m_gripper,
            "AutoPreloadAndLeaveCommunityLong", m_autoTrajectory);
        break;
      case AUTOPRECHARGE :
        m_autoCommand = new AutoPreloadAndEngageChargeStation(m_swerve, m_elbow, m_extension, m_wrist, m_gripper,
            "AutoPreloadMidAndEngageChargeStation", m_autoTrajectory);
        break;
      case AUTOTESTPATH :
        m_autoCommand = new AutoDrivePath(m_swerve, "DriveExample", m_autoTrajectory, false);
        break;
      case AUTOSHORTNOTE :
        m_autoCommand = new AutoDrivePath(m_swerve, pathName, m_autoTrajectory, true);
        break;
    }

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

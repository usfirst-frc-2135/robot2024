
// ROBOTBUILDER TYPE: Robot.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot
{
  private final RobotContainer m_robotContainer = new RobotContainer( );
  private Command              m_autonomousCommand;
  private boolean              m_faultsCleared  = false;

  /**
   * This function is run when the robot is first started up and should be used for any initialization
   * code.
   */
  @Override
  public void robotInit( )
  {
    // Starts recording to data log
    DataLogManager.start( );

    CommandScheduler.getInstance( ).onCommandInitialize(cmd -> DataLogManager.log(String.format("%s: Init", cmd.getName( ))));
    CommandScheduler.getInstance( ).onCommandInterrupt(cmd -> DataLogManager.log(String.format("%s: Interrupt", cmd.getName( ))));
    CommandScheduler.getInstance( ).onCommandFinish(cmd -> DataLogManager.log(String.format("%s: End", cmd.getName( ))));

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic( )
  {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running 
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    CommandScheduler.getInstance( ).run( );
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit( )
  {
    DataLogManager.log(String.format("disabledInit: Match %s%s, %s Alliance", DriverStation.getMatchType( ).toString( ),
        DriverStation.getMatchNumber( ), DriverStation.getAlliance( ).toString( )));

    m_robotContainer.initialize( );
    //m_robotContainer.m_power.initialize( );
  }

  /**
   * This function is called periodically while disabled.
   */
  @Override
  public void disabledPeriodic( )
  {
    // If RoboRIO User button is pressed, dump all CAN faults
    if (RobotController.getUserButton( ))
    {
      if (!m_faultsCleared)
      {
        m_faultsCleared = true;
        m_robotContainer.faultDump( );
      }
    }
    else
      m_faultsCleared = false;
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit( )
  {
    DataLogManager.log(String.format("autonomousInit: Match %s%s, %s Alliance", DriverStation.getMatchType( ).toString( ),
        DriverStation.getMatchNumber( ), DriverStation.getAlliance( ).toString( )));

    m_autonomousCommand = m_robotContainer.getAutonomousCommand( );

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule( );
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic( )
  {}

  /**
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit( )
  {
    DataLogManager.log(String.format("teleopInit: Match %s%s, %s Alliance", DriverStation.getMatchType( ).toString( ),
        DriverStation.getMatchNumber( ), DriverStation.getAlliance( ).toString( )));

    // This makes sure that the autonomous stops running when teleop starts running. 

    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel( );
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic( )
  {}

  /**
   * This function is called once each time the robot enters Simulation mode.
   */
  @Override
  public void simulationInit( )
  {}

  /**
   * This function is called periodically during simulation.
   */
  @Override
  public void simulationPeriodic( )
  {}

  @Override
  public void testInit( )
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance( ).cancelAll( );
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic( )
  {}

}

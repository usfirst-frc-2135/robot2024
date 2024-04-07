
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * Climber calibration command
 */
public class ClimberCalibrate extends Command
{
  private static final double kTimeout         = 1.0;

  private Climber             m_climber;
  private Timer               m_calibrateTimer = new Timer( );

  /**
   * Command the climber subsystem to do a calibration
   * 
   * @param climber
   */
  public ClimberCalibrate(Climber climber)
  {
    m_climber = climber;

    setName("ClimberCalibrate");
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Start up", getName( )));
    m_calibrateTimer.restart( );
    m_climber.climberCalibrateInit( );
    DataLogManager.log(String.format("%s: Start FPGATime %.3f", getName( ), Timer.getFPGATimestamp( )));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    DataLogManager
        .log(String.format("%s: End FPGATime %.3f (%.3f)", getName( ), Timer.getFPGATimestamp( ), m_calibrateTimer.get( )));
    m_calibrateTimer.stop( );
    m_climber.climberCalibrateEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_calibrateTimer.hasElapsed(kTimeout) || m_climber.climberCalibrateIsFinished( ));
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

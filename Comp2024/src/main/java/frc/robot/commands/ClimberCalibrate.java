
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * Climber calibration command
 */
public class ClimberCalibrate extends Command
{
  private Climber m_climber;

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
    m_climber.climberCalibrateInit( );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_climber.climberCalibrateEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return m_climber.climberCalibrateIsFinished( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

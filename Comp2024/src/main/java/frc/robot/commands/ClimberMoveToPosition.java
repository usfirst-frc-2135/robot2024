
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 *
 */
public class ClimberMoveToPosition extends Command
{
  private final Climber m_climber;
  private boolean       m_holdPosition;
  private double        m_newLength = 0.0;

  // Default command for holding current position
  public ClimberMoveToPosition(Climber climber)
  {
    m_climber = climber;
    m_holdPosition = true;
    ClimberMoveToPositionCommon(true);
  }

  // Motion Magic movement to a new position
  public ClimberMoveToPosition(Climber climber, double position)
  {
    m_climber = climber;
    m_holdPosition = false;
    m_newLength = position;
    ClimberMoveToPositionCommon(false);
  }

  private void ClimberMoveToPositionCommon(boolean holdCurrentLength)
  {
    setName("ClimberMoveToPosition");
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_climber.moveToPositionInit(m_newLength, m_holdPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_climber.moveToPositionExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_climber.moveToPositionEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_holdPosition) ? false : m_climber.moveToPositionIsFinished( ); // Command exits if not holding a position
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

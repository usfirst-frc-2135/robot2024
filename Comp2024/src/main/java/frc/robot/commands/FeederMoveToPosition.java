
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;

/**
 *
 */
public class FeederMoveToPosition extends Command
{
  private final Feeder m_feeder;
  private boolean      m_holdPosition;
  private double       m_newLength = 0.0;

  // Default command for holding current position
  public FeederMoveToPosition(Feeder feeder)
  {
    m_feeder = feeder;
    m_holdPosition = true;
    FeederMoveToPositionCommon(true);
  }

  // Motion Magic movement to a new position
  public FeederMoveToPosition(Feeder feeder, double position)
  {
    m_feeder = feeder;
    m_holdPosition = false;
    m_newLength = position;
    FeederMoveToPositionCommon(false);
  }

  private void FeederMoveToPositionCommon(boolean holdCurrentLength)
  {
    setName("ClimberMoveToPosition");
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_feeder.moveToPositionInit(m_newLength, m_holdPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_feeder.moveToPositionExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_feeder.moveToPositionEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_holdPosition) ? false : m_feeder.moveToPositionIsFinished( ); // Command exits if not holding a position
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

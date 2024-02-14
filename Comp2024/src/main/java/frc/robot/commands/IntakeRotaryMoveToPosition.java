// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeRotaryMoveToPosition extends Command
{
  private final Intake m_intake;
  private boolean      m_holdPosition;
  private double       m_newAngle;

  // Default command for holding current position
  public IntakeRotaryMoveToPosition(Intake intake)
  {
    m_intake = intake;
    IntakeRotaryMoveToPositionCommon(true);
  }

  // Motion Magic movement to a new position
  public IntakeRotaryMoveToPosition(Intake intake, double position)
  {
    m_intake = intake;
    m_newAngle = position;
    IntakeRotaryMoveToPositionCommon(false);
  }

  private void IntakeRotaryMoveToPositionCommon(boolean holdCurrentAngle)
  {
    m_holdPosition = holdCurrentAngle;
    setName("IntakeRotaryMoveToPosition");
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_intake.moveToPositionInit(m_newAngle, m_holdPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_intake.moveToPositionExecute( );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_intake.moveToPositionEnd( );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return (m_holdPosition) ? false : m_intake.moveToPositionIsFinished( ); // Command exits if not holding a position
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

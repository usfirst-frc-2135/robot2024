//
// Intake Roller Run command - sets motors to desired mode
//
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INConsts.RollerMode;
import frc.robot.subsystems.Intake;

//
// Intake Roller Run command
//
public class IntakeRollerRun extends Command
{
  // Member variables/objects
  private final Intake     m_intake;
  private final RollerMode m_mode;
  private boolean          m_holdPosition;
  private double           m_newAngle;

  public IntakeRollerRun(Intake intake, RollerMode mode)
  {
    m_intake = intake;
    m_mode = mode;

    IntakeRotaryMoveToPositionCommon(true);
  }

  public IntakeRollerRun(Intake intake, RollerMode mode, double position)
  {
    m_intake = intake;
    m_mode = mode;
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
    m_intake.setRollerSpeed(m_mode);
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

//
// Intake Run command - sets roller and rotary motors to desired mode
//
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INConsts.RollerMode;
import frc.robot.subsystems.Intake;

//
// Intake Run command
//
public class IntakeRun extends Command
{
  // Member variables/objects
  private final Intake     m_intake;
  private final RollerMode m_mode;
  private final boolean    m_holdAngle;
  private double           m_newAngle = 0.0;

  public IntakeRun(Intake intake, RollerMode mode)
  {
    m_intake = intake;
    m_mode = mode;
    m_holdAngle = true;

    IntakeRunCommon( );
  }

  public IntakeRun(Intake intake, RollerMode mode, double newAngle)
  {
    m_intake = intake;
    m_mode = mode;
    m_holdAngle = false;
    m_newAngle = newAngle;

    IntakeRunCommon( );
  }

  private void IntakeRunCommon( )
  {
    setName("IntakeRun");
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_intake.setRollerSpeed(m_mode);
    m_intake.moveToPositionInit(m_newAngle, m_holdAngle);
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
    return (m_holdAngle) ? true : m_intake.moveToPositionIsFinished( ); // Command exits if not holding a position
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

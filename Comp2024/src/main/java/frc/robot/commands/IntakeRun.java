//
// Intake Run command - sets roller and rotary motors to desired mode
//
package frc.robot.commands;

import java.util.function.DoubleSupplier;

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
  private final boolean    m_holdPosition;
  private DoubleSupplier   m_getPosition = null;

  /**
   * Control intake rotary and roller motors
   * 
   * @param intake
   *          intake subsystem
   * @param mode
   *          roller mode desired
   * @param getPosition
   *          rotary position desired
   * @param hold
   *          if true hold the current position
   */
  public IntakeRun(Intake intake, RollerMode mode, DoubleSupplier getPosition, boolean hold)
  {
    m_intake = intake;
    m_mode = mode;
    m_holdPosition = hold;
    m_getPosition = getPosition;

    IntakeRunCommon( );
  }

  /**
   * Control intake rotary and roller motors
   * 
   * @param intake
   *          intake subsystem
   * @param mode
   *          roller mode desired
   * @param getPosition
   *          rotary position desired
   */
  public IntakeRun(Intake intake, RollerMode mode, DoubleSupplier getPosition)
  {
    m_intake = intake;
    m_mode = mode;
    m_holdPosition = false;
    m_getPosition = getPosition;

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
    m_intake.setRollerMode(m_mode);
    m_intake.moveToPositionInit(m_getPosition.getAsDouble( ), m_holdPosition);
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

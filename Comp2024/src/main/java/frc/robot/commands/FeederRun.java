//
// Feeder Run command - sets roller and rotary motors to desired mode
//
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FDConsts.FDRollerMode;
import frc.robot.subsystems.Feeder;

//
// Feeder Run command
//
public class FeederRun extends Command
{
  // Member variables/objects
  private final Feeder       m_feeder;
  private final FDRollerMode m_mode;
  private final boolean      m_holdPosition;
  private DoubleSupplier     m_getPosition = null;

  public FeederRun(Feeder feeder, FDRollerMode mode, DoubleSupplier getPosition, boolean hold)
  {
    m_feeder = feeder;
    m_mode = mode;
    m_holdPosition = hold;
    m_getPosition = getPosition;

    FeederRunCommon( );
  }

  public FeederRun(Feeder feeder, FDRollerMode mode, DoubleSupplier getPosition)
  {
    m_feeder = feeder;
    m_mode = mode;
    m_holdPosition = false;
    m_getPosition = getPosition;

    FeederRunCommon( );
  }

  private void FeederRunCommon( )
  {
    setName("FeederRun");
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_feeder.setRollerSpeed(m_mode);
    m_feeder.moveToPositionInit(m_getPosition.getAsDouble( ), m_holdPosition);
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

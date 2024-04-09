
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

/**
 * Feeder MoveWithJoystick command
 */
public class FeederMoveWithJoystick extends Command
{

  private final Feeder         m_feeder;
  private final DoubleSupplier m_getAxis;

  /**
   * Command the feeder manually using a joystick axis
   * 
   * @param feeder
   *          feeder subsystem
   * @param getAxis
   *          double supplier that retrieves joystick axis
   */
  public FeederMoveWithJoystick(Feeder feeder, DoubleSupplier getAxis)
  {
    m_feeder = feeder;
    m_getAxis = getAxis;

    setName("FeederMoveWithJoystick");
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_feeder.moveRotaryWithJoystick(m_getAxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return false;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

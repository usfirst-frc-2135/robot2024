
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Intake MoveWithJoystick command
 */
public class IntakeMoveWithJoystick extends Command
{

  private final Intake         m_intake;
  private final DoubleSupplier m_getAxis;

  /**
   * Command the intake manually using a joystick axis
   * 
   * @param intake
   *          intake subsystem
   * @param getAxis
   *          double supplier that retrieves joystick axis
   */
  public IntakeMoveWithJoystick(Intake intake, DoubleSupplier getAxis)
  {
    m_intake = intake;
    m_getAxis = getAxis;

    setName("IntakeMoveWithJoystick");
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_intake.moveRotaryWithJoystick(m_getAxis);
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


// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

/**
 *
 */
public class FeederMoveWithJoystick extends Command
{

  private final Feeder   m_feeder;
  private XboxController m_gamePad;

  public FeederMoveWithJoystick(Feeder feeder, XboxController gamePad)
  {
    m_feeder = feeder;
    m_gamePad = gamePad;

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
    m_feeder.moveRotaryWithJoystick(-m_gamePad.getRightY( ));
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

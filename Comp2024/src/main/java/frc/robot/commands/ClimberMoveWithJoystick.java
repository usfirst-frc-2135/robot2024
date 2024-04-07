
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * Climber MoveWithJoystick command
 */
public class ClimberMoveWithJoystick extends Command
{
  private final Climber  m_climber;

  private XboxController m_gamePad;

  /**
   * Command the climber manually using a joystick axis
   * 
   * @param climber
   *          climber subsystem
   * @param gamePad
   *          gamepad to select the joystick from
   */
  public ClimberMoveWithJoystick(Climber climber, XboxController gamePad)
  {
    m_climber = climber;
    m_gamePad = gamePad;

    setName("ClimberMoveWithJoystick");
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_climber.moveWithJoystick(-m_gamePad.getRightY( ));
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

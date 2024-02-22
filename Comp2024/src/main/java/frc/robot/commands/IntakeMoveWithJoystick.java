package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeMoveWithJoystick extends Command
{

  private final Intake   m_intake;
  private XboxController m_gamePad;

  public IntakeMoveWithJoystick(Intake intake, XboxController gamePad)
  {
    m_intake = intake;
    m_gamePad = gamePad;

    setName("IntakeRotaryJoysticks");
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
    m_intake.moveRotaryWithJoystick(-m_gamePad.getRightY( ));
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

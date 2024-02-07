package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.Constants.INConsts.RotaryMode;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeRotaryJoysticks extends Command
{

  private final Intake   m_rotary;
  private XboxController m_gamePad;

  public IntakeRotaryJoysticks(Intake rotary, XboxController gamePad)
  {
    m_rotary = rotary;
    m_gamePad = gamePad;

    setName("IntakeRotaryJoysticks");
    addRequirements(m_rotary);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_rotary.moveRotaryWithJoystick(m_gamePad);
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


// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SHConsts.ShooterMode;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class ShooterRun extends Command
{
  private final Shooter     m_shooter;
  private final ShooterMode m_mode;

  public ShooterRun(Shooter shooter, ShooterMode mode)
  {
    m_shooter = shooter;
    m_mode = mode;

    setName("ShooterRun");
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_shooter.setShooterMode(m_mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return true;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}


// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 *
 */
public class AutoStop extends Command
{
  private final CommandSwerveDrivetrain m_drivetrain;

  public AutoStop(CommandSwerveDrivetrain drivetrain)
  {
    m_drivetrain = drivetrain;

    setName("AutoStop");
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {
    m_drivetrain.setControl(new SwerveRequest.Idle( ));
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

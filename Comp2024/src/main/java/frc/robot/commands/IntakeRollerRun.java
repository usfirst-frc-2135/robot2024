//
// Intake Roller Run command - sets motors to desired mode
//
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.subsystems.Intake;

//
// Intake Roller Run command
//
public class IntakeRollerRun extends Command
{
  // Member variables/objects
  private final Intake       m_intake;
  private final INRollerMode m_mode;

  public IntakeRollerRun(Intake intake, INRollerMode mode)
  {
    m_intake = intake;
    m_mode = mode;

    setName("IntakeRollerRun");
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    m_intake.setIntakeRollerSpeed(m_mode);
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

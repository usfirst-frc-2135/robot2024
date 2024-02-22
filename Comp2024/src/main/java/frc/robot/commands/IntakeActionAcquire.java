
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeActionAcquire extends SequentialCommandGroup
{
  public IntakeActionAcquire(Intake intake)
  {
    setName("IntakeActionAcquire");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Start rollers & Deploy intake rotary"),
        new IntakeRun(intake, INConsts.RollerMode.ACQUIRE, INConsts.kRotaryAngleDeployed).asProxy(),

        new PrintCommand(getName() + ": Wait for note"),
        new WaitUntilCommand(intake::isNoteDetected),

        new PrintCommand(getName() + ": Stop rollers & Retract intake rotary"),
        new IntakeRun(intake, INConsts.RollerMode.STOP, INConsts.kRotaryAngleRetracted).asProxy()
 
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

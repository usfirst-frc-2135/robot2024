
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
public class IntakingAction extends SequentialCommandGroup
{
  public IntakingAction(Intake intake)
  {
    setName("IntakingAction");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Start rollers"),
        new PrintCommand(getName() + ": Deploy intake"),
        new IntakeRollerRun(intake, INConsts.RollerMode.ACQUIRE, Intake.kAngleDeployed).asProxy(),

        new PrintCommand(getName() + ": Wait for note"),        
        new WaitUntilCommand(intake::isNoteDetected),

        new PrintCommand(getName() + ": Stop rollers"),
        new PrintCommand(getName() + ": Retract intake"),

        new IntakeRollerRun(intake, INConsts.RollerMode.STOP, Intake.kAngleRetracted).asProxy()
 
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

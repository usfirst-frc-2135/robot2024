
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeActionRetract extends SequentialCommandGroup
{
  public IntakeActionRetract(Intake intake)
  {
    setName("IntakeActionRetract");

    addCommands(
        // Add Commands here:

        // @formatter:off
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

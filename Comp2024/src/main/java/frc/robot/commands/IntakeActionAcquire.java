
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 *
 */
public class IntakeActionAcquire extends SequentialCommandGroup
{
  public IntakeActionAcquire(Intake intake, LED led)
  {
    setName("IntakeActionAcquire");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Start rollers & Deploy intake rotary"),
        new LEDSet(led, LEDColor.YELLOW, LEDAnimation.CLEARALL),
        new IntakeRun(intake, INConsts.RollerMode.ACQUIRE, INConsts.kRotaryAngleDeployed),

        new PrintCommand(getName() + ": Wait for note"),
        new WaitUntilCommand(intake::isNoteDetected),

        new PrintCommand(getName() + ": Stop rollers & Retract intake rotary"),
        new IntakeRun(intake, INConsts.RollerMode.STOP, INConsts.kRotaryAngleRetracted)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

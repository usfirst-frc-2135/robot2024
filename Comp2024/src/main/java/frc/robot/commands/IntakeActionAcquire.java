
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        new LogCommand(getName(), "Start rollers & Deploy intake rotary"),
        new LEDSet(led, LEDColor.YELLOW, LEDAnimation.CLEARALL),
        new IntakeRun(intake, INConsts.RollerMode.ACQUIRE, intake::getIntakeDeployed),

        new LogCommand(getName(), "Wait for note"),
        new WaitUntilCommand(intake::isNoteDetected),
  
        
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getIntakeRetracted),
        new WaitCommand(0.1),
        new IntakeRun(intake, INConsts.RollerMode.EXPEL, intake::getRotaryPosition),
        new WaitCommand(0.1),
        new IntakeRun(intake, INConsts.RollerMode.ACQUIRE, intake::getRotaryPosition),
        new WaitCommand(0.1),

        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getRotaryPosition)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

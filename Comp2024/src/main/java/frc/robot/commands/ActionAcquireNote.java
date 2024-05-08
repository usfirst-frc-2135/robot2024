
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.ANIMATION;
import frc.robot.Constants.LEDConsts.COLOR;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 * Command to acquire a note from floor
 */
public class ActionAcquireNote extends SequentialCommandGroup
{
  /**
   * Group command to use the intake to acquire a note from the floor
   * 
   * @param intake
   *          intake subsystem
   * @param led
   *          led subsystem
   */
  public ActionAcquireNote(Intake intake, LED led)
  {
    setName("ActionAcquireNote");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Start rollers & Deploy intake rotary"),
        led.getLEDCommand(COLOR.YELLOW, ANIMATION.CLEARALL),
        intake.getMoveToPositionCommand(INConsts.RollerMode.ACQUIRE, intake::getIntakeDeployed),

        new LogCommand(getName(), "Wait for note"),
        new WaitUntilCommand(intake::isNoteDetected),

        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
        intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, intake::getIntakeRetracted)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

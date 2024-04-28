
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 * Command to expel a note to the floor
 */
public class ActionExpelNote extends SequentialCommandGroup
{
  /**
   * Group command to use the intake to expel a note to the floor
   * 
   * @param intake
   *          intake subsystem
   * @param led
   *          led subsystem
   */
  public ActionExpelNote(Intake intake, LED led)
  {
    setName("ActionExpelNote");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Stop rollers & Deploy intake rotary"),
        intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, intake::getIntakeDeployed),

        new LogCommand(getName(), "Expel rollers & Hold intake rotary in same position"),        
        intake.getMoveToPositionCommand(INConsts.RollerMode.EXPEL, intake::getIntakePosition),

        new LogCommand(getName(), "Wait for note to release"),
        new WaitCommand(0.5),

        new LogCommand(getName(), "Stop rollers & Hold intake rotary in same position"),
        intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, intake::getIntakePosition),
        led.getLEDCommand(LEDColor.OFF, LEDAnimation.CLEARALL)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

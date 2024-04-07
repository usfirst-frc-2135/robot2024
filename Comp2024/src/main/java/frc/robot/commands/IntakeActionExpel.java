
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
 * Intake ActionExpel
 */
public class IntakeActionExpel extends SequentialCommandGroup
{
  /**
   * Group command to use the intake to expel a note to the floor
   * 
   * @param intake
   *          intake subsystem
   * @param led
   *          led subsystem
   */
  public IntakeActionExpel(Intake intake, LED led)
  {
    setName("IntakeActionExpel");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Stop rollers & Deploy intake rotary"),
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getIntakeDeployed),

        new LogCommand(getName(), "Expel rollers & Hold intake rotary in same position"),        
        new IntakeRun(intake, INConsts.RollerMode.EXPEL, intake::getIntakePosition),

        new LogCommand(getName(), "Wait for note to release"),
        new WaitCommand(0.5),

        new LogCommand(getName(), "Stop rollers & Hold intake rotary in same position"),
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getIntakePosition),
        new LEDSet(led, LEDColor.OFF, LEDAnimation.CLEARALL)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

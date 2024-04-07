
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 * Intake ActionRetract command
 */
public class IntakeActionRetract extends SequentialCommandGroup
{
  /**
   * Group command to move the intake to retracted position
   * 
   * @param intake
   *          intake subsystem
   * @param led
   *          led subsystem
   */
  public IntakeActionRetract(Intake intake, LED led)
  {
    setName("IntakeActionRetract");

    addCommands(
        // Add Commands here:

        // @formatter:off      
        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
        new ConditionalCommand(
          new LEDSet(led, LEDColor.BLUE, LEDAnimation.CLEARALL), 
          new LEDSet(led, LEDColor.OFF, LEDAnimation.CLEARALL), 
          intake::isNoteDetected),
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getIntakeRetracted)
        
        //@formatter:on
    );

  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

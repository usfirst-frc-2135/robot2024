
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 *
 */
public class IntakeActionRetract extends SequentialCommandGroup
{
  public IntakeActionRetract(Intake intake, LED led)
  {
    setName("IntakeActionRetract");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Stop rollers & Retract intake rotary"),
        new IntakeRun(intake, INConsts.RollerMode.STOP, INConsts.kRotaryAngleRetracted),

        new PrintCommand(getName() + ": Turn CANdle blue"),
        new LEDSet(led, LEDColor.BLUE, LEDAnimation.CLEARALL)
 
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}


// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class ShooterActionFire extends SequentialCommandGroup
{
  public ShooterActionFire(Shooter shooter, Intake intake, LED led)
  {
    setName("ShooterActionFire");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new PrintCommand(getName() + ": Start shooter and retract intake"),
        // new ShooterRun(shooter, ShooterMode.SCORE),  // Already running
        new LEDSet(led, LEDColor.RED, LEDAnimation.CLEARALL),

        new PrintCommand(getName() + ": Wait for desired speed"),
        new WaitUntilCommand(shooter::isAtDesiredSpeed),

        new PrintCommand(getName() + ": Feed note from intake"),
        new LEDSet(led, LEDColor.GREEN, LEDAnimation.CLEARALL),
        new IntakeActionShoot(intake, led),

        new LEDSet(led, LEDColor.OFF, LEDAnimation.CLEARALL),
        // new ShooterRun(shooter, ShooterMode.STOP), // Don't turn off
        new IntakeActionRetract(intake, led)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

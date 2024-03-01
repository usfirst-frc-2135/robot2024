
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.SHConsts.ShooterMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class ShooterActionFire extends SequentialCommandGroup
{
  public ShooterActionFire(Shooter shooter, Intake intake)
  {
    setName("ShooterActionFire");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Start shooter and retract intake"),
        new ShooterRun(shooter, ShooterMode.SCORE),
        new IntakeActionRetract(intake),

        new PrintCommand(getName() + ": Wait for desired speed"),
        new WaitUntilCommand(shooter::isAtDesiredSpeed),

        new PrintCommand(getName() + ": Feed note from intake"),
        new IntakeRun(intake, INConsts.RollerMode.SHOOT, INConsts.kRotaryAngleRetracted),

        new WaitCommand(2.0),
        new ShooterRun(shooter, ShooterMode.STOP),
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

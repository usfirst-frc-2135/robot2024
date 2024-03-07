
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SHConsts.ShooterMode;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class AutoPreload extends ParallelCommandGroup
{
  public AutoPreload(CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter)
  {
    setName("AutoPreload");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": AutoStop and Shoot Preload"),
        new SequentialCommandGroup (
          new ShooterRun(shooter, ShooterMode.SCORE),
          new IntakeActionShoot(intake)
        ) 
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

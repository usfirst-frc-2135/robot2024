
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.SHConsts.ShooterMode;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

/**
 *
 */
public class AutoPreload extends SequentialCommandGroup
{
  public AutoPreload(CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter, LED led)
  {
    setName("AutoPreload");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Spin up shooter"),
        new ShooterRun(shooter, ShooterMode.SCORE),

        new PrintCommand(getName() + ": Wait to be at speed"),
        new WaitUntilCommand(shooter::isAtDesiredSpeed),

        new PrintCommand(getName() + ": Shoot note, wait a bit, and stop shooter"),
        new IntakeActionShoot(intake, led),
        new WaitCommand(0.5),
        
        new LEDSet(led, LEDColor.GREEN, LEDAnimation.CLEARALL),
        new ShooterRun(shooter, ShooterMode.STOP)
 
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

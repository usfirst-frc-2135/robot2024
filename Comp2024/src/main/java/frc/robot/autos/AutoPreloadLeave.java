
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActionScoreSpeaker;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

/**
 * Command to acquire a note from floor
 */
public class AutoPreloadLeave extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Drive to a scoring position
   * 2 - Shoot the preloaded note
   * 3 - Leave the starting zone while avoiding spike notes
   * 
   * @param ppAuto
   *          swerve drivetrain subsystem
   * @param drivetrain
   *          swerve drivetrain subsystem
   * @param intake
   *          intake subsystem
   * @param shooter
   *          shooter subsystem
   * @param led
   *          led subsystem
   */
  public AutoPreloadLeave(List<PathPlannerPath> ppAuto, CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter,
      LED led)
  {
    setName("AutoPreloadLeave");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Drive to scoring pose"),
        drivetrain.getPathCommand(ppAuto.get(0).toString()),

        new LogCommand(getName(), "Score preloaded note"),
        new ActionScoreSpeaker(shooter, intake, led),

        new LogCommand(getName(), "Leave zone"),
        drivetrain.getPathCommand(ppAuto.get(1).toString())
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

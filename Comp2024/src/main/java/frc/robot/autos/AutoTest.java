
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;

/**
 * Command to acquire a note from floor
 */
public class AutoTest extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Run an auto with a test path
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
  public AutoTest(List<PathPlannerPath> ppAuto, CommandSwerveDrivetrain drivetrain, LED led)
  {
    setName("AutoTest");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Drive a test path"),
        drivetrain.getPathCommand(ppAuto.get(0).toString())
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}


package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INConsts;
import frc.robot.commands.AcquireNote;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

/**
 * Auto command that shoots preloaded note and steals notes from centerline
 */
public class AutoPreloadSteal extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Drive to a scoring position
   * 2 - Shoot the preloaded note
   * 3 - Drive to steal centerline notes
   * 
   * @param ppPaths
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
  public AutoPreloadSteal(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter,
      LED led, HID hid)
  {
    setName("AutoPreloadSteal");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Drive to scoring pose"),
        drivetrain.getPathCommand(ppPaths.get(0)),

        new LogCommand(getName(), "Score preloaded note"),
        new ScoreSpeaker(shooter, intake, led),

        new LogCommand(getName(), "Drive to centerline acquire note 1"),
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(
            drivetrain.getPathCommand(ppPaths.get(1)),
            drivetrain.getPathCommand(ppPaths.get(2))
          ),
          new AcquireNote(intake, led, hid)
        ),
        new ConditionalCommand(
          new ScoreSpeaker(shooter, intake, led),
          new LogCommand(getName(), "Missed centerline 1 note"),
          intake::isNoteDetected
        ),

        new LogCommand(getName(), "Drive to centerline acquire note 2"),
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(
            drivetrain.getPathCommand(ppPaths.get(3)),
            drivetrain.getPathCommand(ppPaths.get(4))
          ),
          new AcquireNote(intake, led, hid)
        ),
        new ConditionalCommand(
          new ScoreSpeaker(shooter, intake, led),
          new LogCommand(getName(), "Missed centerline 2 note"),
          intake::isNoteDetected
        ),

        new LogCommand(getName(), "Turn off intake rollers"), 
        intake.getMoveToPositionCommand(INConsts.INRollerMode.STOP, intake::getCurrentPosition)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

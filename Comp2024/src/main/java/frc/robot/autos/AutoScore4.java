
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.INConsts;
import frc.robot.commands.ActionAcquireNote;
import frc.robot.commands.ActionScoreSpeaker;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

/**
 * Command to acquire a note from floor
 */
public class AutoScore4 extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Drive to a scoring position
   * 2 - Shoot the preloaded note
   * 3a - Start and deploy intake
   * 3b - Drive to a spike while acquiring a note
   * 3c - Drive to a scoring position
   * 3d - Shoot a the first spike note
   * 4a - Start and deploy intake
   * 4b - Drive to a spike while acquiring a note
   * 4c - Drive to a scoring position
   * 4d - Shoot a the second spike note
   * 5a - Start and deploy intake
   * 5b - Drive to a spike while acquiring a note
   * 5c - Drive to a scoring position
   * 5d - Shoot a the third spike note
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
  public AutoScore4(List<PathPlannerPath> ppAuto, CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter, LED led)
  {
    setName("AutoScore4");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Drive to scoring pose"),
        drivetrain.getAutoCommand(ppAuto.get(0).toString()),

        new LogCommand(getName(), "Score preloaded note"),
        new ActionScoreSpeaker(shooter, intake, led),

        new LogCommand(getName(), "Deploy intake before moving"),
        intake.getMoveToPositionCommand(INConsts.RollerMode.ACQUIRE, intake::getIntakeDeployed),

        new WaitCommand(0.5),  // TODO - do we need this? The intake command will run to completion first

        new LogCommand(getName(), "Drive to spike while intaking"),
        new ParallelCommandGroup( 
            drivetrain.getAutoCommand(ppAuto.get(1).toString()),
            new ActionAcquireNote(intake, led).withTimeout(1.5)
        ),

        new LogCommand(getName(), "Drive to scoring pose"),
        drivetrain.getAutoCommand(ppAuto.get(2).toString()),

        new LogCommand(getName(), "Score note 2"),
        new ActionScoreSpeaker(shooter, intake, led),

        new LogCommand(getName(), "Drive to spike while intaking"),
        new ParallelCommandGroup(
            drivetrain.getAutoCommand(ppAuto.get(3).toString()),
            new ActionAcquireNote(intake, led).withTimeout(1.5)
        ),

        new LogCommand(getName(), "Drive to scoring pose"),
        drivetrain.getAutoCommand(ppAuto.get(4).toString()),

        new LogCommand(getName(), "Score note 3"),
        new ActionScoreSpeaker(shooter, intake, led),

        new LogCommand(getName(), "Drive to spike while intaking"),
        new ParallelCommandGroup(
            drivetrain.getAutoCommand(ppAuto.get(5).toString()),
            new ActionAcquireNote(intake, led).withTimeout(1.5)
        ), 
        
        new LogCommand(getName(), "Drive to scoring pose"),
        drivetrain.getAutoCommand(ppAuto.get(6).toString()),

        new LogCommand(getName(), "Score note 4"),
        new ActionScoreSpeaker(shooter, intake, led),

        new LogCommand(getName(), "Turn off intake rollers"), 
        intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, intake::getIntakePosition)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

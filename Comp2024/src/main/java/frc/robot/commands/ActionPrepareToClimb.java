
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FDConsts;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;

/**
 * Command to prepare for a climb
 */
public class ActionPrepareToClimb extends SequentialCommandGroup
{
  /**
   * Group command to prepare for a climb
   * 
   * @param climber
   *          climber subsystem
   * @param feeder
   *          feeder subsystem
   */
  public ActionPrepareToClimb(Climber climber, Feeder feeder)
  {
    setName("ActionPrepareToClimb");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new ParallelCommandGroup( 
          feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.STOP, feeder::getFeederAmp),
          climber.getMoveToPositionCommand(climber::getClimberFullyExtended)
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

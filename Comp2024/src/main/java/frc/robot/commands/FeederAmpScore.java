
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FDConsts;
import frc.robot.subsystems.Feeder;

/**
 * Feeder AmpScore command
 */
public class FeederAmpScore extends SequentialCommandGroup
{
  /**
   * Group command to use the feeder to score a note in the amp
   * 
   * @param feeder
   *          feeder subsystem
   */
  public FeederAmpScore(Feeder feeder)
  {
    setName("FeederHandoff");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Align Feeder to Amp"),
        new FeederRun(feeder, FDConsts.FDRollerMode.STOP, feeder::getFeederAmp),

        new LogCommand(getName(), "Score Note to Amp"),
        new FeederRun(feeder, FDConsts.FDRollerMode.SCORE, feeder::getFeederPosition),
        new WaitCommand(0.5),

        new LogCommand(getName(), "Stop rollers"),
        new FeederRun(feeder, FDConsts.FDRollerMode.STOP, feeder::getFeederHandoff)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

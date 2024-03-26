
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FDConsts;
import frc.robot.subsystems.Feeder;

/**
 *
 */
public class FeederAmpScore extends SequentialCommandGroup
{
  public FeederAmpScore(Feeder feeder)
  {
    setName("FeederHandoff");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Align Feeder to Amp"),
        new FeederRun(feeder, FDConsts.FDRollerMode.STOP, feeder::getRotaryAmp),

        new LogCommand(getName(), "Score Note to Amp"),
        new FeederRun(feeder, FDConsts.FDRollerMode.SCORE, feeder::getRotaryPosition),
        new WaitCommand(0.5),

        new LogCommand(getName(), "Stop rollers"),
        new FeederRun(feeder, FDConsts.FDRollerMode.STOP, feeder::getRotaryHandoff)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

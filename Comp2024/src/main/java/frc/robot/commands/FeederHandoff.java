
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FDConsts;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class FeederHandoff extends SequentialCommandGroup
{
  public FeederHandoff(Intake intake, Feeder feeder)
  {
    setName("FeederHandoff");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Align Feeder and Intake"),
        new FeederRun(feeder, FDConsts.FDRollerMode.STOP, feeder::getRotaryHandoff),
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getIntakeHandoff),

        new LogCommand(getName(), "Transfer Note"),
        new FeederRun(feeder, FDConsts.FDRollerMode.HANDOFF, feeder::getRotaryPosition),
        new WaitCommand(0.2),
        new IntakeRun(intake, INConsts.RollerMode.HANDOFF, intake::getRotaryPosition),
        new WaitCommand(0.5),

        new LogCommand(getName(), "Stop rollers"),
        new FeederRun(feeder, FDConsts.FDRollerMode.STOP, feeder::getRotaryPosition),

        new LogCommand(getName(), "Ensure Intake releases Note"),
        new WaitCommand(0.2),
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getIntakeRetracted)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

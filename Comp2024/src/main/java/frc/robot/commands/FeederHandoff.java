
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
        new FeederRun(feeder, FDConsts.FDRollerMode.STOP, FDConsts.kRotaryAngleHandoff),
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getIntakeHandoff),

        new LogCommand(getName(), "Transfer Note"),
        new FeederRun(feeder, FDConsts.FDRollerMode.ACQUIRE),
        new IntakeRun(intake, INConsts.RollerMode.EXPEL),
        new WaitCommand(1.0),

        new LogCommand(getName(), "Stop rollers"),
        new FeederRun(feeder, FDConsts.FDRollerMode.STOP),
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getRotaryPosition)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

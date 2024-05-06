
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FDConsts;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

/**
 * Command to handoff from intake to feeder
 */
public class ActionHandoff extends SequentialCommandGroup
{
  /**
   * Group command to handoff from intake to feeder
   * 
   * @param intake
   *          intake subsystem
   * @param feeder
   *          feeder subsystem
   */
  public ActionHandoff(Intake intake, Feeder feeder)
  {
    setName("ActionHandoff");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Align Feeder and Intake"),
        feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.HANDOFF, feeder::getFeederHandoff),
        intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, intake::getIntakeHandoff),

        new LogCommand(getName(), "Transfer Note"),
        feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.HOLD, feeder::getFeederPosition),
        intake.getMoveToPositionCommand(INConsts.RollerMode.HANDOFF, intake::getIntakePosition),
        
        new WaitCommand(0.1),

        new LogCommand(getName(), "Stop rollers"),
        feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.STOP, feeder::getFeederPosition),

        new WaitCommand(0.2),

        new LogCommand(getName(), "Ensure Intake releases Note"),
        intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, intake::getIntakeRetracted)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
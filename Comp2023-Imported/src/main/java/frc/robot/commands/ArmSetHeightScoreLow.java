
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.EXConsts;
import frc.robot.Constants.WRConsts;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class ArmSetHeightScoreLow extends SequentialCommandGroup
{
  public ArmSetHeightScoreLow(Elbow elbow, Extension extension, Wrist wrist)
  {
    setName("ArmSetHeightScoreLow");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Retract Extension"),
        new ExtensionMoveToPosition(extension, elbow, EXConsts.kLengthIdle).asProxy(),

        new ConditionalCommand(
          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToPosition(elbow,  ELConsts.kAngleScoreLow).asProxy(),

            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToPosition(wrist, WRConsts.kAngleScoreLow).asProxy()
          ),

          new SequentialCommandGroup(
            new PrintCommand(getName() + ": Move Wrist"),
            new WristMoveToPosition(wrist, WRConsts.kAngleScoreLow).asProxy(),

            new PrintCommand(getName() + ": Move Elbow"),
            new ElbowMoveToPosition(elbow,  ELConsts.kAngleScoreLow).asProxy()
          ),

          elbow::isBelowLow
        ),

        new PrintCommand(getName() + ": Extend Extension"),
        new ExtensionMoveToPosition(extension, elbow, EXConsts.kLengthScoreLow).asProxy()
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

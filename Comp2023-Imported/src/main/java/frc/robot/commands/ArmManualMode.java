
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class ArmManualMode extends SequentialCommandGroup
{
  public ArmManualMode(Elbow elbow, Extension extension, Wrist wrist, XboxController operPad)
  {
    setName("ArmManualMode");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Activated"),
        new ParallelCommandGroup(
          new ElbowRun(elbow, operPad),
          new ExtensionRun(extension, operPad),
          new WristRun(wrist, operPad)
        ),

        new PrintCommand(getName() + ": Off")
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

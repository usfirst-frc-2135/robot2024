
// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * A command that logs a string when initialized.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class LogCommand extends InstantCommand
{
  /**
   * Creates a new a LogCommand.
   *
   * @param prefix
   *          the prefix before the message
   * @param message
   *          the message to print
   */
  public LogCommand(String prefix, String msg)
  {
    DataLogManager.log(String.format("%s: %s", prefix, msg));
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return true;
  }
}

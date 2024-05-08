
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

/**
 * Command to shoot into the speaker
 */
public class ActionScoreSpeaker extends SequentialCommandGroup
{
  /**
   * Group command to fire a note to speaker
   * 
   * @param shooter
   *          shooter subsystem
   * @param intake
   *          intake subsystem
   * @param led
   *          led subsystem
   */
  public ActionScoreSpeaker(Shooter shooter, Intake intake, LED led)
  {
    setName("ActionScoreSpeaker");

    addCommands(
        // Add Commands here:

        // @formatter:off
        
        new LogCommand(getName(), "Start shooter, stop rollers and retract intake"),
        // shooter.getShooterScoreCommand(), // Already running
        intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, intake::getIntakeRetracted),

        new LogCommand(getName(), "Wait for desired speed"),
        new WaitUntilCommand(shooter::isAtTargetRPM),

        new LogCommand(getName(), "Feed note from intake"),

        new LogCommand(getName(), "Expel rollers & Hold intake rotary in same position"),            
        intake.getMoveToPositionCommand(INConsts.RollerMode.SHOOT, intake::getIntakePosition),

        new LogCommand(getName(), "Wait for note to release"),
        new WaitCommand(0.5),

        new LogCommand(getName(), "Stop rollers & Hold intake rotary in same position"),
        intake.getMoveToPositionCommand(INConsts.RollerMode.STOP, intake::getIntakePosition)

        // shooter.getShooterStoppedCommand(), // Don't turn off

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

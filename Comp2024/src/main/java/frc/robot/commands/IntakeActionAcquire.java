
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 * Intake ActionAcquire command
 */
public class IntakeActionAcquire extends SequentialCommandGroup
{
  /**
   * Group command to use the intake to acquire a note from the floor
   * 
   * @param intake
   *          intake subsystem
   * @param led
   *          led subsystem
   */
  public IntakeActionAcquire(Intake intake, LED led)
  {
    setName("IntakeActionAcquire");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Start rollers & Deploy intake rotary"),
        new LEDSet(led, LEDColor.YELLOW, LEDAnimation.CLEARALL),
        new IntakeRun(intake, INConsts.RollerMode.ACQUIRE, intake::getIntakeDeployed),

        new LogCommand(getName(), "Wait for note"),
        new WaitUntilCommand(intake::isNoteDetected),
  
        
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getIntakeRetracted),
        new WaitCommand(0.1),
        new IntakeRun(intake, INConsts.RollerMode.EXPEL, intake::getIntakePosition),
        new WaitCommand(0.1),
        new IntakeRun(intake, INConsts.RollerMode.ACQUIRE, intake::getIntakePosition),
        new WaitCommand(0.1),

        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
        new IntakeRun(intake, INConsts.RollerMode.STOP, intake::getIntakePosition)
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

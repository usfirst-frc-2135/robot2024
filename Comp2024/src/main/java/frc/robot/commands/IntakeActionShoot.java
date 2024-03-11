
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 *
 */
public class IntakeActionShoot extends SequentialCommandGroup
{
  public IntakeActionShoot(Intake intake, LED led)
  {
    setName("IntakeActionShoot");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Hold rollers & Retract intake rotary"),
        new IntakeRun(intake, INConsts.RollerMode.HOLD, INConsts.kRotaryAngleRetracted),

        new PrintCommand(getName() + ": Expel rollers & Hold intake rotary in same position"),            
        new LEDSet(led, LEDColor.GREEN, LEDAnimation.CLEARALL),
        new IntakeRun(intake, INConsts.RollerMode.SHOOT),

        new PrintCommand(getName() + ": Wait for note to release"),
        new WaitCommand(0.5),

        new PrintCommand(getName() + ": Stop rollers & Hold intake rotary in same position"),
        new IntakeRun(intake, INConsts.RollerMode.HOLD)
 
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeActionHandoff extends SequentialCommandGroup
{
  public IntakeActionHandoff(Intake intake)
  {
    setName("IntakeActionHandoff");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Hold rollers & Retract intake rotary"),
        new IntakeRun(intake, INConsts.RollerMode.HOLD, INConsts.kRotaryAngleHandoff),

        new PrintCommand(getName() + ": Expel rollers & Hold intake rotary in same position"),
            
        new IntakeRun(intake, INConsts.RollerMode.HANDOFF, intake.getIntakePosition( )),

        new PrintCommand(getName() + ": Wait for note to release into feeder"),
        new WaitCommand(0.5),

        new PrintCommand(getName() + ": Stop rollers & Hold intake rotary in same position"),
        new IntakeRun(intake, INConsts.RollerMode.HOLD, intake.getIntakePosition( ))
 
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

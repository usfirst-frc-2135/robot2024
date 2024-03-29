
// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GRConsts.GRMode;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 *
 */
public class AutoPreloadAndLeaveCommunityLong extends SequentialCommandGroup
{
  public AutoPreloadAndLeaveCommunityLong(Swerve swerve, Elbow elbow, Extension extension, Wrist wrist, Gripper gripper,
      String pathName, PathPlannerTrajectory trajectory)
  {
    setName("AutoPreloadAndLeaveCommunityLong");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new PrintCommand(getName() + ": Score Preload"),        
        new ExtensionCalibrate(extension).asProxy(), 
        new AutoPreload(elbow, extension, wrist, gripper),

        new GripperRun(gripper, GRMode.GR_STOP),
        
        new PrintCommand(getName() + ": Drive Off Community"),
        new AutoDrivePath(swerve, pathName, trajectory, true),

        new PrintCommand(getName() + ": Hold in place"),
        new AutoStop(swerve)
        // @formatter:on
    );

  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}

//
// LED Set command - sets CANdle to desired mode
//
package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConsts.AnimationTypes;
import frc.robot.Constants.LEDConsts.LEDColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LED;

/**
 *
 */
public class LEDSet extends Command
{
  private final LEDColor       m_color;
  private final LED            m_led;
  private final AnimationTypes m_animation;

  public LEDSet(LED led, LEDColor color, AnimationTypes animation) //AnimationTypes animation)
  {
    m_led = led;
    m_color = color;
    m_animation = animation;

    setName("LEDSet");
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize( )
  {
    DataLogManager.log("Entered initialize");

    if (m_animation == AnimationTypes.SetAll)
    {

      m_led.setColor(m_color);
      DataLogManager.log("\tSETCOLOR");

    }
    else if (m_color == LEDColor.LEDCOLOR_OFF)
    {
      m_led.setAnimation(m_animation);
      DataLogManager.log("SETANIMATION");
    }
    else
    {
      DataLogManager.log("else");
      m_led.setColor(LEDColor.LEDCOLOR_OFF);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( )
  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished( )
  {
    return true;
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return true;
  }
}

//
// LED subystem - LED feedback on robot
//
package frc.robot.subsystems;

import javax.xml.crypto.Data;

import org.ejml.dense.row.decomposition.svd.SafeSvd_DDRM;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.Ports;

//
// LED subsystem class
//
public class LED extends SubsystemBase
{
  // Member objects
  private final CANdle                          m_candle              = new CANdle(Ports.kCANID_CANdle);
  private final int                             LedCount              = 8;
  private final SendableChooser<LEDColor>       m_ledChooser          = new SendableChooser<LEDColor>( );
  private final SendableChooser<AnimationTypes> m_ledAnimationChooser = new SendableChooser<AnimationTypes>( );
  private LEDColor                              m_previousColor       = LEDColor.LEDCOLOR_OFF;
  private Animation                             m_toAnimate           = null;

  public enum AnimationTypes
  {
    ColorFlow, Fire, Larson, Rainbow, RgbFade, SingleFade, Strobe, Twinkle, TwinkleOff, SetAll, AnimationDash
  }

  private AnimationTypes m_currentAnimation;

  // Constructor
  public LED( )
  {
    setName("LED");
    setSubsystem("LED");

    //setColor(LEDColor.LEDCOLOR_BLUE);
    m_candle.configBrightnessScalar(0.7);

    // Add options for colors in SmartDashboard
    m_ledChooser.setDefaultOption("LED_Off", LEDColor.LEDCOLOR_OFF);
    m_ledChooser.addOption("LED_White", LEDColor.LEDCOLOR_WHITE);
    m_ledChooser.addOption("LED_Red", LEDColor.LEDCOLOR_RED);
    m_ledChooser.addOption("LED_Orange", LEDColor.LEDCOLOR_ORANGE);
    m_ledChooser.addOption("LED_Yellow", LEDColor.LEDCOLOR_YELLOW);
    m_ledChooser.addOption("LED_Green", LEDColor.LEDCOLOR_GREEN);
    m_ledChooser.addOption("LED_Blue", LEDColor.LEDCOLOR_BLUE);
    m_ledChooser.addOption("LED_Purple", LEDColor.LEDCOLOR_PURPLE);

    m_ledAnimationChooser.setDefaultOption("SetAll", AnimationTypes.SetAll);
    m_ledAnimationChooser.addOption("ColorFlow", AnimationTypes.ColorFlow);
    m_ledAnimationChooser.addOption("Fire", AnimationTypes.Fire);
    m_ledAnimationChooser.addOption("Larson", AnimationTypes.Larson);
    m_ledAnimationChooser.addOption("Rainbow", AnimationTypes.Rainbow);
    m_ledAnimationChooser.addOption("RgbFade", AnimationTypes.RgbFade);
    m_ledAnimationChooser.addOption("SingleFade", AnimationTypes.SingleFade);
    m_ledAnimationChooser.addOption("Strobe", AnimationTypes.Strobe);
    m_ledAnimationChooser.addOption("Twinkle", AnimationTypes.Twinkle);
    m_ledAnimationChooser.addOption("TwinkleOff", AnimationTypes.TwinkleOff);

    SmartDashboard.putData("LED_Color", m_ledChooser);
    SmartDashboard.putData("LED_Animation", m_ledAnimationChooser);

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic( )
  {

  }

  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setColor(LEDColor.LEDCOLOR_DASH);

  }

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }

  public void setColor(LEDColor color)
  {
    final String strName;

    if (m_previousColor != color)
    {
      if (color == (LEDColor.LEDCOLOR_DASH))
        color = m_ledChooser.getSelected( );
      // color = LEDColor.LEDCOLOR_PURPLE;

      switch (color)
      {
        default :
        case LEDCOLOR_WHITE :
          strName = "WHITE";
          m_candle.setLEDs(255, 255, 255); // white
          break;
        case LEDCOLOR_OFF :
          strName = "OFF";
          m_candle.setLEDs(0, 0, 0); // black
          break;
        case LEDCOLOR_RED :
          strName = "RED";
          m_candle.setLEDs(255, 0, 0); // red
          break;
        case LEDCOLOR_ORANGE :
          strName = "ORANGE";
          m_candle.setLEDs(255, 80, 0); // orange
          break;
        case LEDCOLOR_YELLOW :
          strName = "YELLOW";
          m_candle.setLEDs(255, 255, 0); // yellow
          break;
        case LEDCOLOR_GREEN :
          strName = "GREEN";
          m_candle.setLEDs(0, 255, 0); // green
          break;
        case LEDCOLOR_BLUE :
          strName = "BLUE";
          m_candle.setLEDs(0, 0, 255); // blue
          break;
        case LEDCOLOR_PURPLE :
          strName = "PURPLE";
          m_candle.setLEDs(144, 0, 255); // purple
          break;
      }
      DataLogManager.log(String.format("%s: COLOR IS NOW %s", getSubsystem( ), strName));
      m_previousColor = color;

    }
  }

  public void setAnimation(AnimationTypes animation)
  {
    final String strName;

    // if (animation == (AnimationTypes.AnimationDash))
    //   animation = m_ledAnimationChooser.getSelected( );
    // //m_candle.animate(AnimationTypes.animation);
    // DataLogManager.log("Animation Selected!");

    switch (animation)
    {
      default :
      case ColorFlow :
        strName = "ColorFlow";
        m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
        break;
      case Fire :
        strName = "Fire";
        m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
        break;
      case Larson :
        strName = "Larson";
        m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
        break;
      case Rainbow :
        strName = "Rainbow";
        m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
        break;
      case RgbFade :
        strName = "RgbFade";
        m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
        break;
      case SingleFade :
        strName = "SingleFade";
        m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
        break;
      case Strobe :
        strName = "Strobe";
        m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
        break;
      case Twinkle :
        strName = "Twinkle";
        m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent64);
        break;
      case TwinkleOff :
        strName = "TwinkleOff";
        m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
        break;
      case SetAll :
        strName = "SetAll";
        m_toAnimate = null;
        break;
    }
    m_candle.animate(m_toAnimate);
    DataLogManager.log(String.format("%s: ANIMATION IS NOW %s", getSubsystem( ), strName));
    //m_previousColor = color;
  }
}

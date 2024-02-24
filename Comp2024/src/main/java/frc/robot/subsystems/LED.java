//
// LED subystem - LED feedback on robot
//
package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConsts.Animations;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.Ports;

//
// LED subsystem class
//
public class LED extends SubsystemBase
{
  // Member objects
  private final CANdle                      m_candle              = new CANdle(Ports.kCANID_CANdle);
  private final int                         LedCount              = 8;
  private final SendableChooser<LEDColor>   m_ledChooser          = new SendableChooser<LEDColor>( );
  private final SendableChooser<Animations> m_ledAnimationChooser = new SendableChooser<Animations>( );
  private LEDColor                          m_previousColor       = LEDColor.LEDCOLOR_OFF;
  private Animation                         m_toAnimate           = null;

  private class myColor
  {
    private int r;
    private int g;
    private int b;

    private myColor(int red, int green, int blue)
    {
      r = red;
      g = green;
      b = blue;
    }
  }

  private myColor color_white  = new myColor(255, 255, 255);
  private myColor color_red    = new myColor(255, 0, 0);
  private myColor color_orange = new myColor(255, 80, 0);
  private myColor color_yellow = new myColor(255, 255, 0);
  private myColor color_green  = new myColor(0, 255, 0);
  private myColor color_blue   = new myColor(0, 0, 255);
  private myColor color_purple = new myColor(144, 0, 255);
  private myColor color_off    = new myColor(0, 0, 0);

  private myColor ChosenColor;

  // private AnimationTypes m_currentAnimation;

  public LED( )
  {
    setName("LED");
    setSubsystem("LED");

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

    // Animation options in Smart Dashboard
    m_ledAnimationChooser.setDefaultOption("SetAll", Animations.SETALL);
    m_ledAnimationChooser.addOption("ColorFlow", Animations.COLORFLOW);
    m_ledAnimationChooser.addOption("Fire", Animations.FIRE);
    m_ledAnimationChooser.addOption("Larson", Animations.LARSON);
    m_ledAnimationChooser.addOption("Rainbow", Animations.RAINBOW);
    m_ledAnimationChooser.addOption("RgbFade", Animations.RGBFADE);
    m_ledAnimationChooser.addOption("SingleFade", Animations.SINGLEFADE);
    m_ledAnimationChooser.addOption("Strobe", Animations.STROBE);
    m_ledAnimationChooser.addOption("Twinkle", Animations.TWINKLE);
    m_ledAnimationChooser.addOption("TwinkleOff", Animations.TWINKLEOFF);

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
    //setColor(LEDColor.LEDCOLOR_DASH);

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
      DataLogManager.log("Color Selected");
      //color = LEDColor.LEDCOLOR_PURPLE;

      switch (color)
      {
        default :
        case LEDCOLOR_WHITE :
          ChosenColor = color_white;
          strName = "WHITE";
          m_candle.setLEDs(color_white.r, color_white.g, color_white.b); // white
          break;
        case LEDCOLOR_OFF :
          ChosenColor = color_off;
          strName = "OFF";
          m_candle.setLEDs(color_off.r, color_off.g, color_off.b); // black
          break;
        case LEDCOLOR_RED :
          ChosenColor = color_red;
          strName = "RED";
          m_candle.setLEDs(color_red.r, color_red.g, color_red.b); // red
          break;
        case LEDCOLOR_ORANGE :
          ChosenColor = color_orange;
          strName = "ORANGE";
          m_candle.setLEDs(color_orange.r, color_orange.g, color_orange.b); // orange
          break;
        case LEDCOLOR_YELLOW :
          ChosenColor = color_yellow;
          strName = "YELLOW";
          m_candle.setLEDs(color_yellow.r, color_yellow.g, color_yellow.b); // yellow
          break;
        case LEDCOLOR_GREEN :
          ChosenColor = color_green;
          strName = "GREEN";
          m_candle.setLEDs(color_green.r, color_green.g, color_green.b); // green
          break;
        case LEDCOLOR_BLUE :
          ChosenColor = color_blue;
          strName = "BLUE";
          m_candle.setLEDs(color_blue.r, color_blue.g, color_blue.b); // blue
          break;
        case LEDCOLOR_PURPLE :
          ChosenColor = color_purple;
          strName = "PURPLE";
          m_candle.setLEDs(color_purple.r, color_purple.g, color_purple.b); // purple
          break;
      }
      DataLogManager.log(String.format("%s: COLOR IS NOW %s", getSubsystem( ), strName));
      m_previousColor = color;

    }
  }

  public void setAnimation(Animations animation)
  {
    final String strName;
    //String Color;

    if (animation == (Animations.ANIMATIONDASH))
      animation = m_ledAnimationChooser.getSelected( );
    DataLogManager.log("Animation Selected!");

    switch (animation)
    {
      default :
      case COLORFLOW :
        strName = "ColorFlow"; //single color
        m_toAnimate = new ColorFlowAnimation(ChosenColor.r, ChosenColor.g, ChosenColor.b, 0, 0.7, LedCount, Direction.Forward);
        break;
      case FIRE :
        strName = "Fire";
        m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
        break;
      case LARSON :
        strName = "Larson"; //single color
        m_toAnimate = new LarsonAnimation(ChosenColor.r, ChosenColor.g, ChosenColor.b, 0, 1, LedCount, BounceMode.Front, 3);
        break;
      case RAINBOW :
        strName = "Rainbow";
        m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
        break;
      case RGBFADE :
        strName = "RgbFade";
        m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
        break;
      case SINGLEFADE :
        strName = "SingleFade"; //single color
        m_toAnimate = new SingleFadeAnimation(ChosenColor.r, ChosenColor.g, ChosenColor.b, 0, 0.5, LedCount);
        break;
      case STROBE :
        strName = "Strobe"; //single color
        m_toAnimate = new StrobeAnimation(ChosenColor.r, ChosenColor.g, ChosenColor.b, 0, 98.0 / 256.0, LedCount);
        break;
      case TWINKLE :
        strName = "Twinkle"; //single color
        m_toAnimate =
            new TwinkleAnimation(ChosenColor.r, ChosenColor.g, ChosenColor.b, 0, 0.4, LedCount, TwinklePercent.Percent64);
        break;
      case TWINKLEOFF :
        strName = "TwinkleOff";
        m_toAnimate =
            new TwinkleOffAnimation(ChosenColor.r, ChosenColor.g, ChosenColor.b, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
        break;
      case SETALL :
        strName = "SetAll";
        m_toAnimate = null;
        break;
    }
    DataLogManager.log(String.format("%s: ANIMATION IS NOW %s", getSubsystem( ), strName));
    //m_previousColor = color;
    m_candle.animate(m_toAnimate);
  }
}

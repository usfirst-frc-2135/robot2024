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
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.Ports;

//
// LED subsystem class
//
public class LED extends SubsystemBase
{
  // Member objects
  private final CANdle                        m_candle              = new CANdle(Ports.kCANID_CANdle);
  private final int                           LedCount              = 8;
  private final SendableChooser<LEDColor>     m_ledChooser          = new SendableChooser<LEDColor>( );
  private final SendableChooser<LEDAnimation> m_ledAnimationChooser = new SendableChooser<LEDAnimation>( );
  private LEDColor                            m_previousColor       = LEDColor.LEDCOLOR_OFF;
  private Animation                           m_toAnimate           = null;

  public enum LEDAnimation
  {
    COLORFLOW,    // Slow flowing color
    FIRE,         // Flickering single color from one end of string
    LARSON,       // Ping-pong end-to-end
    RAINBOW,      // Flowing rainbow
    RGBFADE,      // Fade red, then green, then blue
    SINGLEFADE,   // Single color fade in and out
    STROBE,       // FLashing strobe of single color
    TWINKLE,      // Twinkle colors on
    TWINKLEOFF,   // Twinkle colors off
    SETALL,       // 
    ANIMATIONDASH // Select animation from dashboard chooser
  }

  private LEDAnimation m_currentAnimation;

  // Constructor
  public LED( )
  {
    setName("LED");
    setSubsystem("LED");

    setColor(LEDColor.LEDCOLOR_BLUE);
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

    m_ledAnimationChooser.setDefaultOption("SetAll", LEDAnimation.SETALL);
    m_ledAnimationChooser.addOption("ColorFlow", LEDAnimation.COLORFLOW);
    m_ledAnimationChooser.addOption("Fire", LEDAnimation.FIRE);
    m_ledAnimationChooser.addOption("Larson", LEDAnimation.LARSON);
    m_ledAnimationChooser.addOption("Rainbow", LEDAnimation.RAINBOW);
    m_ledAnimationChooser.addOption("RgbFade", LEDAnimation.RGBFADE);
    m_ledAnimationChooser.addOption("SingleFade", LEDAnimation.SINGLEFADE);
    m_ledAnimationChooser.addOption("Strobe", LEDAnimation.STROBE);
    m_ledAnimationChooser.addOption("Twinkle", LEDAnimation.TWINKLE);
    m_ledAnimationChooser.addOption("TwinkleOff", LEDAnimation.TWINKLEOFF);

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
    // This method will be called once per scheduler run when in simulation
  }

  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setColor(LEDColor.LEDCOLOR_OFF);

  }

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }

  public void setColor(LEDColor color)
  {
    if (m_previousColor != color)
    {
      if (color == (LEDColor.LEDCOLOR_DASH))
        color = m_ledChooser.getSelected( );

      switch (color)
      {
        default :
        case LEDCOLOR_OFF :
          m_candle.setLEDs(0, 0, 0); // black
          break;
        case LEDCOLOR_WHITE :
          m_candle.setLEDs(255, 255, 255); // white
          break;
        case LEDCOLOR_RED :
          m_candle.setLEDs(255, 0, 0); // red
          break;
        case LEDCOLOR_ORANGE :
          m_candle.setLEDs(255, 48, 0); // orange
          break;
        case LEDCOLOR_YELLOW :
          m_candle.setLEDs(255, 255, 0); // yellow
          break;
        case LEDCOLOR_GREEN :
          m_candle.setLEDs(0, 255, 0); // green
          break;
        case LEDCOLOR_BLUE :
          m_candle.setLEDs(0, 0, 255); // blue
          break;
        case LEDCOLOR_PURPLE :
          m_candle.setLEDs(128, 0, 255); // purple
          break;
      }
      DataLogManager.log(String.format("%s: color is now %s", getSubsystem( ), color));
      m_previousColor = color;

    }
  }

  public void setAnimation(LEDAnimation animation)
  {
    if (animation == LEDAnimation.ANIMATIONDASH)
      animation = m_ledAnimationChooser.getSelected( );

    switch (animation)
    {
      default :
      case COLORFLOW :
        m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
        break;
      case FIRE :
        m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
        break;
      case LARSON :
        m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
        break;
      case RAINBOW :
        m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
        break;
      case RGBFADE :
        m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
        break;
      case SINGLEFADE :
        m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
        break;
      case STROBE :
        m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
        break;
      case TWINKLE :
        m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent64);
        break;
      case TWINKLEOFF :
        m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
        break;
      case SETALL :
        m_toAnimate = null;
        break;
    }

    m_candle.animate(m_toAnimate);
    DataLogManager.log(String.format("%s: animation is now %s", getSubsystem( ), animation));
  }
}

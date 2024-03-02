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
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.Ports;

//
// LED subsystem class
//
public class LED extends SubsystemBase
{
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

  // Constants
  private static final int                    kLEDCount             = 8;

  private final myColor                       kWhite                = new myColor(255, 255, 255);
  private final myColor                       kRed                  = new myColor(255, 0, 0);
  private final myColor                       kOrange               = new myColor(255, 48, 0);
  private final myColor                       kYellow               = new myColor(255, 255, 0);
  private final myColor                       kGreen                = new myColor(0, 255, 0);
  private final myColor                       kBlue                 = new myColor(0, 0, 255);
  private final myColor                       kPurple               = new myColor(128, 0, 128);
  private final myColor                       kColorOff             = new myColor(0, 0, 0);

  // Member objects
  private final CANdle                        m_candle              = new CANdle(Ports.kCANID_CANdle);
  private final SendableChooser<LEDColor>     m_ledChooser          = new SendableChooser<LEDColor>( );
  private final SendableChooser<LEDAnimation> m_ledAnimationChooser = new SendableChooser<LEDAnimation>( );
  private LEDColor                            m_previousColor       = LEDColor.OFF;
  private Animation                           m_toAnimate           = null;

  private myColor                             m_chosenColor;

  // Constructor

  public LED( )
  {
    setName("LED");
    setSubsystem("LED");

    m_candle.configBrightnessScalar(0.7);

    // Add options for colors in SmartDashboard
    m_ledChooser.setDefaultOption("OFF", LEDColor.OFF);
    m_ledChooser.addOption("WHITE", LEDColor.WHITE);
    m_ledChooser.addOption("RED", LEDColor.RED);
    m_ledChooser.addOption("ORANGE", LEDColor.ORANGE);
    m_ledChooser.addOption("YELLOW", LEDColor.YELLOW);
    m_ledChooser.addOption("GREEN", LEDColor.GREEN);
    m_ledChooser.addOption("BLUE", LEDColor.BLUE);
    m_ledChooser.addOption("PURPLE", LEDColor.PURPLE);

    // Animation options in Smart Dashboard
    m_ledAnimationChooser.setDefaultOption("CLEARALL", LEDAnimation.CLEARALL);
    m_ledAnimationChooser.addOption("COLORFLOW", LEDAnimation.COLORFLOW);
    m_ledAnimationChooser.addOption("FIRE", LEDAnimation.FIRE);
    m_ledAnimationChooser.addOption("LARSON", LEDAnimation.LARSON);
    m_ledAnimationChooser.addOption("RAINBOW", LEDAnimation.RAINBOW);
    m_ledAnimationChooser.addOption("RGBFADE", LEDAnimation.RGBFADE);
    m_ledAnimationChooser.addOption("SINGLEFADE", LEDAnimation.SINGLEFADE);
    m_ledAnimationChooser.addOption("STROBE", LEDAnimation.STROBE);
    m_ledAnimationChooser.addOption("TWINKLE", LEDAnimation.TWINKLE);
    m_ledAnimationChooser.addOption("TWINKLEOFF", LEDAnimation.TWINKLEOFF);

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
    // This method will be called once per scheduler run during simulation
  }

  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setLEDs(LEDColor.OFF, LEDAnimation.CLEARALL);

  }

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }

  public void setLEDs(LEDColor color, LEDAnimation animation)
  {
    if (color == LEDColor.DASHBOARD)
      color = m_ledChooser.getSelected( );

    else if (animation == LEDAnimation.DASHBOARD)
    {
      animation = m_ledAnimationChooser.getSelected( );
    }

    switch (color)
    {
      default :
      case OFF :
        m_chosenColor = kColorOff;
        break;
      case WHITE :
        m_chosenColor = kWhite;
        break;
      case RED :
        m_chosenColor = kRed;
        break;
      case ORANGE :
        m_chosenColor = kOrange;
        break;
      case YELLOW :
        m_chosenColor = kYellow;
        break;
      case GREEN :
        m_chosenColor = kGreen;
        break;
      case BLUE :
        m_chosenColor = kBlue;
        break;
      case PURPLE :
        m_chosenColor = kPurple;
        break;
    }

    switch (animation)
    {
      default :
      case COLORFLOW :
        m_toAnimate =
            new ColorFlowAnimation(m_chosenColor.r, m_chosenColor.g, m_chosenColor.b, 0, 0.7, kLEDCount, Direction.Forward);
        break;
      case FIRE :
        m_toAnimate = new FireAnimation(0.5, 0.7, kLEDCount, 0.7, 0.5);
        break;
      case LARSON :
        m_toAnimate =
            new LarsonAnimation(m_chosenColor.r, m_chosenColor.g, m_chosenColor.b, 0, 1, kLEDCount, BounceMode.Front, 3);
        break;
      case RAINBOW :
        m_toAnimate = new RainbowAnimation(1, 0.1, kLEDCount);
        break;
      case RGBFADE :
        m_toAnimate = new RgbFadeAnimation(0.7, 0.4, kLEDCount);
        break;
      case SINGLEFADE :
        m_toAnimate = new SingleFadeAnimation(m_chosenColor.r, m_chosenColor.g, m_chosenColor.b, 0, 0.5, kLEDCount);
        break;
      case STROBE :
        m_toAnimate = new StrobeAnimation(m_chosenColor.r, m_chosenColor.g, m_chosenColor.b, 0, 98.0 / 256.0, kLEDCount);
        break;
      case TWINKLE :
        m_toAnimate =
            new TwinkleAnimation(m_chosenColor.r, m_chosenColor.g, m_chosenColor.b, 0, 0.4, kLEDCount, TwinklePercent.Percent64);
        break;
      case TWINKLEOFF :
        m_toAnimate = new TwinkleOffAnimation(m_chosenColor.r, m_chosenColor.g, m_chosenColor.b, 0, 0.8, kLEDCount,
            TwinkleOffPercent.Percent100);
        break;
      case CLEARALL :
        m_toAnimate = null;
        break;
    }

    DataLogManager.log(String.format("%s: color is now %s, %s", getSubsystem( ), color, animation));
    m_candle.animate(m_toAnimate);
    if (animation == LEDAnimation.CLEARALL)
    {
      m_candle.setLEDs(m_chosenColor.r, m_chosenColor.g, m_chosenColor.b);
    }
    m_previousColor = color;
  }
}

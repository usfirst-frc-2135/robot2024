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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConsts.LEDAnimation;
import frc.robot.Constants.LEDConsts.LEDColor;
import frc.robot.Constants.Ports;
import frc.robot.lib.phoenix.PhoenixUtil5;

/****************************************************************************
 * 
 * LED subsystem class
 */
public class LED extends SubsystemBase
{
  private class rgbColor
  {
    private int r;
    private int g;
    private int b;

    private rgbColor(int red, int green, int blue)
    {
      r = red;
      g = green;
      b = blue;
    }
  }

  // Constants
  private static final int                    kLEDCount             = 8;

  private final rgbColor                      kRgbWhite             = new rgbColor(255, 255, 255);
  private final rgbColor                      kRgbRed               = new rgbColor(255, 0, 0);
  private final rgbColor                      kRgbOrange            = new rgbColor(255, 48, 0);
  private final rgbColor                      kRgbYellow            = new rgbColor(255, 255, 0);
  private final rgbColor                      kRgbGreen             = new rgbColor(0, 255, 0);
  private final rgbColor                      kRgbBlue              = new rgbColor(0, 0, 255);
  private final rgbColor                      kRgbPurple            = new rgbColor(128, 0, 128);
  private final rgbColor                      kRgbOff               = new rgbColor(0, 0, 0);

  private final int                           kWhiteness            = 0;    // White level for LED strings that support one
  private final double                        kBrightness           = 0.7;  // Brightness level 0.0 - 1.0
  private final double                        kSpeed                = 0.5;  // Animation speed 0.0 - 1.0
  private final int                           kSlot                 = 0;

  // Member objects
  private final CANdle                        m_candle              = new CANdle(Ports.kCANID_CANdle);
  private final SendableChooser<LEDColor>     m_ledChooser          = new SendableChooser<LEDColor>( );
  private final SendableChooser<LEDAnimation> m_ledAnimationChooser = new SendableChooser<LEDAnimation>( );

  private rgbColor                            m_rgb                 = kRgbOff;
  private Animation                           m_animation           = null;
  private rgbColor                            m_previousRgb         = kRgbOff;
  private Animation                           m_previousAnimation   = null;

  /****************************************************************************
   * 
   * Constructor
   */
  public LED( )
  {
    setName("LED");
    setSubsystem("LED");

    m_candle.configBrightnessScalar(kBrightness);
    m_candle.clearAnimation(kSlot);

    initSmartDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initSmartDashboard( )
  {
    // Initialize dashboard widgets

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
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setLEDs(LEDColor.OFF, LEDAnimation.CLEARALL);
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil5.getInstance( ).candlePrintFaults(m_candle, "candle");
    m_candle.clearStickyFaults( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set LEDs based on the color and animation requested
   * 
   * @param color
   *          requested color
   * @param animation
   *          requested animation
   */
  private void setLEDs(LEDColor color, LEDAnimation animation)
  {
    if (color == LEDColor.DASHBOARD)
      color = m_ledChooser.getSelected( );

    if (animation == LEDAnimation.DASHBOARD)
      animation = m_ledAnimationChooser.getSelected( );

    switch (color)
    {
      default :
      case OFF :
        m_rgb = kRgbOff;
        break;
      case WHITE :
        m_rgb = kRgbWhite;
        break;
      case RED :
        m_rgb = kRgbRed;
        break;
      case ORANGE :
        m_rgb = kRgbOrange;
        break;
      case YELLOW :
        m_rgb = kRgbYellow;
        break;
      case GREEN :
        m_rgb = kRgbGreen;
        break;
      case BLUE :
        m_rgb = kRgbBlue;
        break;
      case PURPLE :
        m_rgb = kRgbPurple;
        break;
    }

    switch (animation)
    {
      default :
      case CLEARALL :
        m_animation = null;
        break;
      case COLORFLOW :
        m_animation = new ColorFlowAnimation(m_rgb.r, m_rgb.g, m_rgb.b, kWhiteness, kSpeed, kLEDCount, Direction.Forward);
        break;
      case FIRE :
        m_animation = new FireAnimation(kBrightness, kSpeed, kLEDCount, 0.7, 0.5);
        break;
      case LARSON :
        m_animation = new LarsonAnimation(m_rgb.r, m_rgb.g, m_rgb.b, kWhiteness, kSpeed, kLEDCount, BounceMode.Front, 3);
        break;
      case RAINBOW :
        m_animation = new RainbowAnimation(kBrightness, kSpeed, kLEDCount);
        break;
      case RGBFADE :
        m_animation = new RgbFadeAnimation(kBrightness, kSpeed, kLEDCount);
        break;
      case SINGLEFADE :
        m_animation = new SingleFadeAnimation(m_rgb.r, m_rgb.g, m_rgb.b, kWhiteness, kSpeed, kLEDCount);
        break;
      case STROBE :
        m_animation = new StrobeAnimation(m_rgb.r, m_rgb.g, m_rgb.b, kWhiteness, kSpeed, kLEDCount);
        break;
      case TWINKLE :
        m_animation = new TwinkleAnimation(m_rgb.r, m_rgb.g, m_rgb.b, kWhiteness, kSpeed, kLEDCount, TwinklePercent.Percent64);
        break;
      case TWINKLEOFF :
        m_animation =
            new TwinkleOffAnimation(m_rgb.r, m_rgb.g, m_rgb.b, kWhiteness, kSpeed, kLEDCount, TwinkleOffPercent.Percent100);
        break;
    }

    if (m_rgb == m_previousRgb && m_animation == m_previousAnimation)
    {
      DataLogManager.log(String.format("%s: Color and animation already active - %s, %s", getName( ), color, animation));
      return;
    }

    DataLogManager.log(String.format("%s: Color is now %s, %s", getSubsystem( ), color, animation));
    if (m_animation != null)
    {
      m_candle.animate(m_animation, kSlot);
    }
    else
    {
      m_candle.clearAnimation(kSlot);
      m_candle.setLEDs(m_rgb.r, m_rgb.g, m_rgb.b, kWhiteness, 0, kLEDCount);
    }

    m_previousRgb = m_rgb;
    m_previousAnimation = m_animation;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create LED set command
   * 
   * @param color
   *          LED color to display
   * @param animation
   *          LED animation pattern to use
   * @return instant command that changes LEDs
   */
  public Command getLEDCommand(LEDColor color, LEDAnimation animation)
  {
    return new InstantCommand(            // Command that runs exactly once
        ( ) -> setLEDs(color, animation), // Method to call
        this                              // Subsystem requirement
    )                                     //
        .withName("LEDSet")          //
        .ignoringDisable(true);
  }

}

//
// Phoenix 6 initialization utilities
//
package frc.robot.lib.phoenix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/****************************************************************************
 * 
 * Phoenix 6 initialization class
 */
public class PhoenixUtil6
{
  private static PhoenixUtil6 m_instance  = null;
  private static final int    m_retries   = 5;    // Number of version check attempts
  private static final String m_className = "PhoenixUtil6";
  private static double       m_timeout   = 0.100;

  /****************************************************************************
   * 
   * Constructor
   */
  PhoenixUtil6( )
  {}

  /****************************************************************************
   * 
   * Retrieve instance of this class (singleton)
   * 
   * @return reference to this (singleton) class
   */
  public static PhoenixUtil6 getInstance( )
  {
    if (m_instance == null)
      m_instance = new PhoenixUtil6( );

    return m_instance;
  }

  /****************************************************************************
   * 
   * Initialize a Talon FX motor and display the result
   * 
   * @param talonFX
   *          reference to an FX controlled motor
   * @param name
   *          descriptive name of the FX controlled motor
   * @param config
   *          the talon FX configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean talonFXInitialize6(TalonFX talonFX, String name, TalonFXConfiguration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int deviceID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean talonValid = false;

    // Display Talon firmware versions
    deviceID = talonFX.getDeviceID( );

    Timer.delay(0.25);

    // This can take multiple attempts before ready
    for (int i = 0; i < m_retries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = talonFX.getVersion( );
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
        Timer.delay(0.1);
    }

    talonValid = (fwvMajor >= Constants.kPhoenix6MajorVersion);

    if (config != null)
      if ((status = talonFX.getConfigurator( ).apply(config, m_timeout)) != StatusCode.OK)
        DataLogManager.log(String.format("%s: ID %2d - %s talonFX: getConfigurator.apply - %s!", m_className, deviceID, name,
            status.getDescription( )));

    if ((status = talonFX.setControl(new VoltageOut(0))) != StatusCode.OK)
      DataLogManager
          .log(String.format("%s: ID %2d - %s talonFX: setControl - %s!", m_className, deviceID, name, status.getDescription( )));

    // Configure sensor settings
    if ((status = talonFX.setPosition(0.0)) != StatusCode.OK)
      DataLogManager.log(String.format("%s: ID %2d - %s talonFX: setRotorPosition - %s!", m_className, deviceID, name,
          status.getDescription( )));

    talonFX.setSafetyEnabled(false);

    DataLogManager.log(String.format("%s: ID %2d - %s talonFX:    ver: %d.%d.%d.%d is %s!", m_className, deviceID, name, fwvMajor,
        fwvMinor, fwvBugfix, fwvBuild, (talonValid) ? "VALID" : "URESPONSIVE"));

    return talonValid;
  }

  /****************************************************************************
   * 
   * Initialize a CANcoder and display the result
   * 
   * @param canCoder
   *          reference to a CANcoder
   * @param name
   *          descriptive name of the CANcoder
   * @param config
   *          the CANcoder configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean canCoderInitialize6(CANcoder canCoder, String name, CANcoderConfiguration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int deviceID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean canCoderValid = false;

    // Display Talon firmware versions
    deviceID = canCoder.getDeviceID( );

    // This can take multiple attempts before ready
    for (int i = 0; i < m_retries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = canCoder.getVersion( );
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
        Timer.delay(0.1);
    }

    canCoderValid = (fwvMajor >= Constants.kPhoenix6MajorVersion);

    if (config != null)
      if ((status = canCoder.getConfigurator( ).apply(config, m_timeout)) != StatusCode.OK)
        DataLogManager.log(String.format("%s: ID %2d - %s CANcoder: getConfigurator.apply - %s!", m_className, deviceID, name,
            status.getDescription( )));

    DataLogManager.log(String.format("%s: ID %2d - %s CANcoder: ver: %d.%d.%d.%d is %s!", m_className, deviceID, name, fwvMajor,
        fwvMinor, fwvBugfix, fwvBuild, (canCoderValid) ? "VALID" : "URESPONSIVE"));

    return canCoderValid;
  }

  /****************************************************************************
   * 
   * Initialize a Pigeon2 IMU and display the result
   * 
   * @param pigeon2
   *          reference to a pigeon2
   * @param config
   *          the pigeon2 configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean pigeon2Initialize6(Pigeon2 pigeon2, Pigeon2Configuration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int deviceID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean pigeon2Valid = false;

    // Display Talon firmware versions
    deviceID = pigeon2.getDeviceID( );

    Timer.delay(0.25);

    // This can take multiple attempts before ready
    for (int i = 0; i < m_retries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = pigeon2.getVersion( );
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
        Timer.delay(0.1);
    }

    pigeon2Valid = (fwvMajor >= Constants.kPhoenix6MajorVersion);

    if (config != null)
      if ((status = pigeon2.getConfigurator( ).apply(config, m_timeout)) != StatusCode.OK)
        DataLogManager.log(
            String.format("%s: ID %2d - pigeon2: getConfigurator.apply - %s!", m_className, deviceID, status.getDescription( )));

    // Configure sensor settings
    if ((status = pigeon2.setYaw(0.0)) != StatusCode.OK)
      DataLogManager.log(String.format("%s: ID %2d - pigeon2: setYaw - %s!", m_className, deviceID, status.getDescription( )));

    DataLogManager.log(String.format("%s: ID %2d - pigeon2:    ver: %d.%d.%d.%d is %s!", m_className, deviceID, fwvMajor,
        fwvMinor, fwvBugfix, fwvBuild, (pigeon2Valid) ? "VALID" : "URESPONSIVE"));

    return pigeon2Valid;
  }
}

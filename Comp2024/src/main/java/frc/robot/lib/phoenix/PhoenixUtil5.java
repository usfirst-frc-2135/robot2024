//
// Phoenix 5 initialization utilities
//
package frc.robot.lib.phoenix;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/****************************************************************************
 * 
 * Phoenix 5 initialization class
 */
public class PhoenixUtil5
{
  private static PhoenixUtil5 instance    = null;
  private static final int    m_retries   = 5;    // Number of version check attempts
  private static final String m_className = "PhoenixUtil5";

  /****************************************************************************
   * 
   * Constructor
   */
  PhoenixUtil5( )
  {}

  /****************************************************************************
   * 
   * Retrieve instance of this class (singleton)
   * 
   * @return reference to this (singleton) class
   */
  public static PhoenixUtil5 getInstance( )
  {
    if (instance == null)
      instance = new PhoenixUtil5( );

    return instance;
  }

  /****************************************************************************
   * 
   * Check last error code for a desired Talon SRX amotor and display the result
   * 
   * @param talonSRX
   *          reference to an SRX controlled motor
   * @param message
   *          informational string about the error being checked
   * @return the last error code
   */
  public ErrorCode talonSRXCheckError(WPI_TalonSRX talonSRX, String message)
  {
    ErrorCode errorCode = talonSRX.getLastError( );
    int deviceID = talonSRX.getDeviceID( );

    if (errorCode != ErrorCode.OK)
      DataLogManager.log(String.format("%s: ID %2d - Msg: %s - error %d", m_className, deviceID, message, errorCode.value));

    return errorCode;
  }

  /****************************************************************************
   * 
   * Initialize a Talon SRX motor and display the result
   * 
   * @param talonSRX
   *          reference to an SRX controlled motor
   * @param name
   *          descriptive name of the SRX controlled motor
   * @param config
   *          the talon SRX configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean talonSRXInitialize(WPI_TalonSRX talonSRX, String name, TalonSRXConfiguration config)
  {
    ErrorCode error = ErrorCode.OK;
    int deviceID = 0;
    int fwVersion = 0;
    String baseStr = name + " talonSRX:    ";
    boolean talonSRXValid = false;
    boolean initialized = false;

    // Display Talon firmware versions
    deviceID = talonSRX.getDeviceID( );
    error = talonSRXCheckError(talonSRX, baseStr + "getDeviceID error");

    Timer.delay(0.250);

    if (error == ErrorCode.OK)
    {
      // This can take multiple attempts before ready
      for (int i = 0; i < m_retries && fwVersion == 0; i++)
      {
        fwVersion = talonSRX.getFirmwareVersion( );
        error = talonSRXCheckError(talonSRX, baseStr + "getFirmwareVersion error");

        if (error == ErrorCode.OK)
        {
          talonSRXValid = true;
          if (fwVersion < Constants.kPhoenix5MajorVersion)
            DataLogManager.log(String.format("%s: ID %2d - %s - Incorrect FW version: %d - error %d", m_className, deviceID,
                baseStr, (fwVersion / 256), error.value));
          break;
        }

        Timer.delay(0.100);
      }
    }

    if (talonSRXValid)
    {
      baseStr += "ver: " + (fwVersion / 256.0);
      error = talonSRX.configFactoryDefault( );
      if (error != ErrorCode.OK)
        DataLogManager
            .log(String.format("%s: ID %2d - Msg: configFactoryDefault error - %d", m_className, deviceID, error.value));
      else
      {
        initialized = true;
        talonSRX.configAllSettings(config);
        error = talonSRXCheckError(talonSRX, baseStr + "configAllSettings error");
      }
    }

    DataLogManager.log(String.format("%s: ID %2d - %s is %s", m_className, deviceID, baseStr,
        (talonSRXValid && initialized) ? "VALID!" : "UNRESPONSIVE!"));

    return talonSRXValid && initialized;
  }

}

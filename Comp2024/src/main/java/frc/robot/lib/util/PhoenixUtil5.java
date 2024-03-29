
// Phoenix 5 initialization utilities

package frc.robot.lib.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class PhoenixUtil5
{
  private static PhoenixUtil5 instance    = null;
  private static final int    m_retries   = 5;    // Number of version check attempts
  private static final String m_className = "PhoenixUtil5";

  PhoenixUtil5( )
  {}

  public static PhoenixUtil5 getInstance( )
  {
    if (instance == null)
      instance = new PhoenixUtil5( );

    return instance;
  }

  // Talon SRX handler

  public ErrorCode talonSRXCheckError(WPI_TalonSRX motor, String message)
  {
    ErrorCode errorCode = motor.getLastError( );
    int deviceID = motor.getDeviceID( );

    if (errorCode != ErrorCode.OK)
      DataLogManager.log(String.format("%s: ID %2d - Msg: %s - error %d", m_className, deviceID, message, errorCode.value));

    return errorCode;
  }

  public boolean talonSRXInitialize(WPI_TalonSRX talon, String name, TalonSRXConfiguration config)
  {
    ErrorCode error = ErrorCode.OK;
    int deviceID = 0;
    int fwVersion = 0;
    String baseStr = name + " motor:    ";
    boolean talonValid = false;
    boolean initialized = false;

    // Display Talon firmware versions
    deviceID = talon.getDeviceID( );
    error = talonSRXCheckError(talon, baseStr + "getDeviceID error");

    Timer.delay(0.250);

    if (error == ErrorCode.OK)
    {
      // This can take multiple attempts before ready
      for (int i = 0; i < m_retries && fwVersion == 0; i++)
      {
        fwVersion = talon.getFirmwareVersion( );
        error = talonSRXCheckError(talon, baseStr + "getFirmwareVersion error");

        if (error == ErrorCode.OK)
        {
          talonValid = true;
          if (fwVersion < Constants.kPhoenix5MajorVersion)
            DataLogManager.log(String.format("%s: ID %2d - %s - Incorrect FW version: %d - error %d", m_className, deviceID,
                baseStr, (fwVersion / 256), error.value));
          break;
        }

        Timer.delay(0.100);
      }
    }

    if (talonValid)
    {
      baseStr += "ver: " + (fwVersion / 256.0);
      error = talon.configFactoryDefault( );
      if (error != ErrorCode.OK)
        DataLogManager
            .log(String.format("%s: ID %2d - Msg: configFactoryDefault error - %d", m_className, deviceID, error.value));
      else
      {
        initialized = true;
        talon.configAllSettings(config);
        error = talonSRXCheckError(talon, baseStr + "configAllSettings error");
      }
    }

    DataLogManager.log(String.format("%s: ID %2d - %s is %s", m_className, deviceID, baseStr,
        (talonValid && initialized) ? "VALID!" : "UNRESPONSIVE!"));

    return talonValid && initialized;
  }

}

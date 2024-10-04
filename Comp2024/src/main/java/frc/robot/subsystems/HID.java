//
// HID subystem - HID feedback on robot
//
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/****************************************************************************
 * 
 * HID subsystem class to provide rumble command factory
 */
public class HID extends SubsystemBase
{
  // Constants

  // Member objects
  private GenericHID m_driver;
  private GenericHID m_operator;
  private Timer      m_timerDriver   = new Timer( );
  private Timer      m_timerOperator = new Timer( );
  private Boolean    m_driverRumbleOn;
  private Boolean    m_operatorRumbleOn;

  /****************************************************************************
   * 
   * Constructor
   */
  public HID(GenericHID driver, GenericHID operator, Boolean driverRumbleOn, Boolean operatorRumbleOn)
  {
    setName("HID");
    setSubsystem("HID");

    Robot.timeMarker(getName( ) + ": constructor start");

    m_driver = driver;
    m_operator = operator;
    m_driverRumbleOn = driverRumbleOn;
    m_operatorRumbleOn = operatorRumbleOn;

    initDashboard( );
    initialize( );

    Robot.timeMarker(getName( ) + ": constructor end");
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  @Override
  public void periodic( )
  {
    if (m_timerDriver.hasElapsed(1.0) && m_driverRumbleOn)
    {
      setHIDRumbleDriver(false, 0);
    }

    if (m_timerOperator.hasElapsed(1.0) && m_operatorRumbleOn)
    {
      setHIDRumbleOperator(false, 0);
    }
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
  private void initDashboard( )
  {

  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {}

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set HIDs based on the requested intensity and options
   * 
   * @param color
   *          requested color
   * @param animation
   *          requested animation
   * @param intensity
   *          requested rumble strength
   */
  private void setHIDRumbleDriver(boolean driverRumble, double intensity)
  {
    DataLogManager.log(String.format("%s: Rumble driver: %s intensity: %.1f", getName( ), driverRumble, intensity));

    m_driverRumbleOn = driverRumble;

    if (m_driverRumbleOn)
    {
      m_timerDriver.restart( );
    }

    m_driver.setRumble(RumbleType.kBothRumble, (driverRumble) ? intensity : 0.0);
  }

  private void setHIDRumbleOperator(boolean operatorRumble, double intensity)
  {
    DataLogManager.log(String.format("%s operator: %s intensity: %.1f", getName( ), operatorRumble, intensity));

    m_operatorRumbleOn = operatorRumble;

    if (m_operatorRumbleOn)
    {
      m_timerOperator.restart( );
    }

    m_operator.setRumble(RumbleType.kBothRumble, (operatorRumble) ? intensity : 0.0);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create HID set command
   * 
   * @param driverRumble
   *          rumble the driver gamepad
   * @param operatorRumble
   *          rumble the operator gamepad
   * @param intensity
   *          stength of the rumble [0.0 .. 1.0]
   * @return instant command that rumbles the gamppads
   */
  public Command getHIDRumbleCommandDriver(boolean driverRumble, double intensity)
  {

    return new InstantCommand(            // Command that runs exactly once
        ( ) -> setHIDRumbleDriver(driverRumble, intensity), // Method to call
        this                              // Subsystem requirement
    ).withName("HIDRumbleOperator").ignoringDisable(true);

  }

  public Command getHIDRumbleCommandOperator(boolean operatorRumble, double intensity)
  {

    return new InstantCommand(            // Command that runs exactly once
        ( ) -> setHIDRumbleOperator(operatorRumble, intensity), // Method to call
        this                              // Subsystem requirement
    ).withName("HIDRumbleOperator").ignoringDisable(true);

  }
}

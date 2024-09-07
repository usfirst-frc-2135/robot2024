//
// HID subystem - HID feedback on robot
//
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  /****************************************************************************
   * 
   * Constructor
   */
  public HID(GenericHID driver, GenericHID operator)
  {
    setName("HID");
    setSubsystem("HID");

    Robot.timeMarker(getName( ) + ": constructor start");

    m_driver = driver;
    m_operator = operator;

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
  private void initDashboard( )
  {
    // Initialize dashboard widgets
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
  private void setHIDRumble(boolean driverRumble, boolean operatorRumble, double intensity)
  {
    DataLogManager.log(
        String.format("%s: Rumble driver: %s operator: %s intensity: %.1f", getName( ), driverRumble, operatorRumble, intensity));
    m_driver.setRumble(RumbleType.kBothRumble, (driverRumble) ? intensity : 0.0);
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
  public Command getHIDRumbleCommand(boolean driverRumble, boolean operatorRumble, double intensity)
  {
    return new StartEndCommand(            // Command that has start and end conditions
        ( ) -> setHIDRumble(driverRumble, operatorRumble, intensity),         // Method to call at start
        ( ) -> setHIDRumble(Constants.kRumbleOff, Constants.kRumbleOff, 0.0), // Method to call at start
        this                              // Subsystem requirement
    )                                     //
        .withName("HIDRumble")       //
        .ignoringDisable(true);
  }

}

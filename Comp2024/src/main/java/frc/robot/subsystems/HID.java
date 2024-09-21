//
// HID subystem - HID feedback on robot
//
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.HID;

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
  private void setHIDRumbleDriver(boolean driverRumble, double intensity)
  {
    DataLogManager.log(String.format("%s: Rumble driver: %s intensity: %.1f", getName( ), driverRumble, intensity));

    m_driver.setRumble(RumbleType.kBothRumble, (driverRumble) ? intensity : 0.0);
  }

  private void setHIDRumbleOperator(boolean operatorRumble, double intensity)
  {
    DataLogManager.log(String.format("%s operator: %s intensity: %.1f", getName( ), operatorRumble, intensity));

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
    {
      return new InstantCommand(            // Command that runs exactly once
          ( ) -> setHIDRumbleDriver(driverRumble, intensity), // Method to call
          this                              // Subsystem requirement
      ).withName("HIDRumbleOperator").ignoringDisable(true).withTimeout(0.5);
    }
  }

  public Command getHIDRumbleCommandOperator(boolean operatorRumble, double intensity)
  {
    return new InstantCommand(            // Command that runs exactly once
        ( ) -> setHIDRumbleOperator(operatorRumble, intensity), // Method to call
        this                              // Subsystem requirement
    ).withName("HIDRumbleOperator").ignoringDisable(true).withTimeout(0.5);
  }
}

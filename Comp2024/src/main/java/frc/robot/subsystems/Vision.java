//
// Vision Subystem - handle limelight interface
//
package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;

/****************************************************************************
 * 
 * Vision subsystem class
 */
public class Vision extends SubsystemBase
{
  private enum streamMode
  {
    STANDARD(0),    //
    PIPMAIN(1),     //
    PIPSECONDARY(2);

    @SuppressWarnings("unused")
    public final int value;

    private streamMode(int value)
    {
      this.value = value;
    }
  };

  // Constants
  private static final double kAimingKp  = 0.01;
  private static final double kDrivingKp = 0.06;

  // Objects

  // Declare module variables
  private NetworkTable        m_table;            // Network table reference for getting LL values

  private streamMode          m_stream   = streamMode.STANDARD;

  /****************************************************************************
   * 
   * Constructor
   */
  public Vision( )
  {
    setName("Vision");
    setSubsystem("Vision");

    // Get the Network table reference once for all methods
    m_table = NetworkTableInstance.getDefault( ).getTable("limelight");

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
    SmartDashboard.putNumber("VI_tx", m_table.getEntry("tx").getDouble(0.0));
    SmartDashboard.putNumber("VI_ty", m_table.getEntry("ty").getDouble(0.0));
    SmartDashboard.putNumber("VI_ta", m_table.getEntry("ta").getDouble(0.0));
    SmartDashboard.putNumber("VI_ts", m_table.getEntry("ts").getDouble(0.0));
    SmartDashboard.putNumber("VI_tv", m_table.getEntry("tv").getDouble(0.0));
    SmartDashboard.putNumber("VI_tid", m_table.getEntry("tid").getDouble(0.0));
    SmartDashboard.putNumber("VI_tl", m_table.getEntry("tl").getDouble(0.0));
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

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));

    LimelightHelpers.setLEDMode_ForceOff("limelight");
    LimelightHelpers.setStreamMode_PiPSecondary("limelight");

    if (DriverStation.getAlliance( ).equals(Optional.of(DriverStation.Alliance.Red)))
    {
      setPriorityId(4, "RED");
    }
    else if (DriverStation.getAlliance( ).equals(Optional.of(DriverStation.Alliance.Blue)))
    {
      setPriorityId(7, "BLUE");
    }
    else
      DataLogManager.log(String.format("%s: Driver station alliance color NOT SET!", getSubsystem( )));
  }

  /****************************************************************************
   * 
   * rotateCameraStreamMode - rotate through different stream modes
   */
  public void rotateCameraStreamMode( )
  {
    switch (m_stream)
    {
      default :
      case PIPSECONDARY :
        m_stream = streamMode.PIPMAIN;
        LimelightHelpers.setStreamMode_PiPMain("limelight");
        break;
      case PIPMAIN :
        m_stream = streamMode.STANDARD;
        LimelightHelpers.setStreamMode_Standard("limelight");
        break;
      case STANDARD :
        m_stream = streamMode.PIPSECONDARY;
        LimelightHelpers.setStreamMode_PiPSecondary("limelight");
    }
    DataLogManager.log(String.format("%s: Set stream mode (setStreamMode_PiPxxx) %s", getSubsystem( ), m_stream));
  }

  /****************************************************************************
   * 
   * Limelight auto-aiming control for rotational velocity.
   * 
   * @param maxAngularRate
   *          max angular rate to scale against
   */
  public double limelight_aim_proportional(double maxAngularRate)
  {
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kAimingKp;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= maxAngularRate;

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  /****************************************************************************
   * 
   * Limelight auto-ranging control for distance velocity.
   * 
   * @param maxSpeed
   *          max speed to scale against
   */
  public double limelight_range_proportional(double maxSpeed)
  {
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kDrivingKp;

    // convert to meters per second
    targetingForwardSpeed *= maxSpeed;

    // invert since ty is positive when the target is above the crosshair
    targetingForwardSpeed *= -1.0;

    return targetingForwardSpeed;
  }

  ///////////////////////// PRIVATE HELPERS ///////////////////////////////

  /****************************************************************************
   * 
   * Set priorityid and display alliance color
   * 
   * @param id
   *          aprilTag ID to set as priority
   * @param alliance
   *          alliance color string selected
   */
  private void setPriorityId(int id, String alliance)
  {
    DataLogManager.log(String.format("%s: Set priority id %d (%s)", getSubsystem( ), id, alliance));
    LimelightHelpers.setPriorityTagID("limelight", id);
  }

}

//
// Vision Subystem - handle limelight interface
//
package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;

/****************************************************************************
 * 
 * Vision subsystem class
 */
public class Vision extends SubsystemBase
{
  private static final String kVisionTab = "Vision";

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
  private static final double kAimingKp    = 0.01;
  private static final double kDrivingKp   = 0.06;

  // Objects

  // Shuffleboard objects
  ShuffleboardTab             m_visionTab  = Shuffleboard.getTab(kVisionTab);
  ShuffleboardLayout          m_targetList =
      m_visionTab.getLayout("Target", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
  GenericEntry                m_txEntry    = m_targetList.add("tx-horizontal", 0.0).getEntry( );
  GenericEntry                m_tyEntry    = m_targetList.add("ty-vertical", 0.0).getEntry( );
  GenericEntry                m_taEntry    = m_targetList.add("ta-area", 0.0).getEntry( );
  GenericEntry                m_tsEntry    = m_targetList.add("ts-skew", 0.0).getEntry( );

  ShuffleboardLayout          m_statusList =
      m_visionTab.getLayout("Status", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 3);
  GenericEntry                m_tvEntry    = m_statusList.add("tv-valid", false).getEntry( );
  GenericEntry                m_tlEntry    = m_statusList.add("tl-latency", 0.0).getEntry( );
  GenericEntry                m_tidEntry   = m_statusList.add("tid-targetid", 0.0).getEntry( );

  // Declare module variables
  private NetworkTable        m_table      = NetworkTableInstance.getDefault( ).getTable("limelight"); // Network table reference for LL values

  private streamMode          m_stream     = streamMode.STANDARD;

  /****************************************************************************
   * 
   * Constructor
   */
  public Vision( )
  {
    setName("Vision");
    setSubsystem("Vision");

    // Get the Network table reference once for all methods

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
    m_tvEntry.setBoolean(m_table.getEntry("tv").getDouble(0.0) > 0.5);
    m_txEntry.setDouble(m_table.getEntry("tx").getDouble(0.0));
    m_tyEntry.setDouble(m_table.getEntry("ty").getDouble(0.0));
    m_taEntry.setDouble(m_table.getEntry("ta").getDouble(0.0));
    m_tsEntry.setDouble(m_table.getEntry("ts").getDouble(0.0));
    m_tlEntry.setDouble(m_table.getEntry("tl").getDouble(0.0));
    m_tidEntry.setDouble(m_table.getEntry("tid").getDouble(0.0));
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
    DataLogManager.log(String.format("%s: Set AprilTag priority id %d (%s)", getSubsystem( ), id, alliance));
    LimelightHelpers.setPriorityTagID("limelight", id);
  }

}

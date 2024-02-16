// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VIConsts;

/**
 *
 */
public class Vision extends SubsystemBase
{
  // Objects
  public MedianFilter           m_tyfilter      = new MedianFilter(5); // median filter y values to remove outliers (5 sample)
  public MedianFilter           m_tvfilter      = new MedianFilter(5); // median filter v values to remove outliers (5 sample)

  // Declare module variables
  private NetworkTable          m_table;            // Network table reference for getting LL values
  private DoubleArraySubscriber m_botPoseSub;

  private double                m_targetHorizAngle;      // LL Target horizontal Offset from Crosshair to Target (-27 to 27 deg)
  private double                m_targetVertAngle;       // LL Target vertical Offset from Crosshair to Target (-20.5 to 20.5 deg)
  private double                m_targetArea;            // LL Target Area (0% of image to 100% of image)
  private double                m_targetSkew;            // LL Target Skew or rotation (-90 to 0 deg)
  private boolean               m_targetValid;           // LL Target Valid or not
  private double                m_targetLatency;         // LL pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
  private int                   m_targetID;              // ID of the primary in-view AprilTag
  private boolean               m_targetIDFixed = false; // Override periodic reading of limelight targetIDs for testing

  /**
   *
   */
  public Vision( )
  {
    setName("Vision");
    setSubsystem("Vision");

    // Get the Network table reference once for all methods
    m_table = NetworkTableInstance.getDefault( ).getTable("limelight");

    // Put all the needed widgets on the dashboard
    SmartDashboard.putNumberArray("VI_RobotPose", new double[ ] { });
    SmartDashboard.putNumber("VI_area", m_targetArea);
    SmartDashboard.putNumber("VI_skew", m_targetSkew);
    SmartDashboard.putBoolean("VI_valid", m_targetValid);
    SmartDashboard.putNumber("VI_targetLatency", m_targetLatency);

    m_botPoseSub = m_table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[ ] { });

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    m_targetHorizAngle = m_table.getEntry("tx").getDouble(0.0);
    m_targetVertAngle = m_tyfilter.calculate(m_table.getEntry("ty").getDouble(0.0));
    m_targetArea = m_table.getEntry("ta").getDouble(0.0);
    m_targetSkew = m_table.getEntry("ts").getDouble(0.0);
    m_targetValid = m_tvfilter.calculate(m_table.getEntry("tv").getDouble(0.0)) > 0.5;
    if (!m_targetIDFixed)
      m_targetID = (int) m_table.getEntry("tid").getDouble(0.0);
    m_targetLatency = m_table.getEntry("tl").getDouble(0.0);

    getLimelightRawPose( );

    SmartDashboard.putNumber("VI_horizAngle", m_targetHorizAngle);
    SmartDashboard.putNumber("VI_vertAngle", m_targetVertAngle);
    SmartDashboard.putNumber("VI_area", m_targetArea);
    SmartDashboard.putNumber("VI_skew", m_targetSkew);
    SmartDashboard.putBoolean("VI_valid", m_targetValid);
    SmartDashboard.putNumber("VI_targetLatency", m_targetLatency);

  }

  @Override
  public void simulationPeriodic( )
  {

  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));

    setLEDMode(VIConsts.LED_OFF);
    setCameraDisplay(VIConsts.PIP_SECONDARY);
  }

  public double getHorizOffsetDeg( )
  {
    return m_targetHorizAngle;
  }

  public double getVertOffsetDeg( )
  {
    return m_targetVertAngle;
  }

  public double getTargetArea( )
  {
    return m_targetArea;
  }

  public double getTargetSkew( )
  {
    return m_targetSkew;
  }

  public boolean getTargetValid( )
  {
    return m_targetValid;
  }

  public int getTargetID( )
  {
    return m_targetID;
  }

  // Used only for debugging to force a valid target ID

  public void setFixedTargetID(int id)
  {
    m_targetIDFixed = true;
    m_targetID = id;
  }

  public double getTargetLatency( )
  {
    return m_targetLatency;
  }

  // Return the limelight pose if the limelight thinks it is valid
  public Pose2d getLimelightRawPose( )
  {
    double[ ] m_botPoseArray = m_botPoseSub.get( );

    if (m_targetValid && (m_botPoseArray != null))
    {
      // Translate Pose3d sent by the limelight in an array into a Pose2d that we use
      // Array [6] order: Translation (X, Y, Z), Rotation(Roll, Pitch, Yaw)
      double yawDegrees = m_botPoseArray[5] + ((m_botPoseArray[5] < 0) ? 360 : 0);

      Pose2d m_botLLPose =
          new Pose2d(new Translation2d(m_botPoseArray[0], m_botPoseArray[1]), new Rotation2d(Units.degreesToRadians(yawDegrees)));

      double[ ] robotPose = new double[ ]
      {
          m_botPoseArray[0], m_botPoseArray[1], m_botPoseArray[5]
      };

      SmartDashboard.putNumberArray("VI_RobotPose", robotPose);

      return m_botLLPose;
    }

    return null;
  }

  // Return the limelight pose if it passes all sanity checks
  public Pose2d getLimelightValidPose(Pose2d currentPose)
  {
    Pose2d llPose = getLimelightRawPose( );

    // If sanity checks are valid, return the limelight pose
    if ((llPose != null) && isLimelightPoseValid(llPose, currentPose))
    {
      return llPose;
    }

    return null;
  }

  // Make sanity checks on the limelight pose
  //  - compare to see if within a specified distance of the current pose
  public boolean isLimelightPoseValid(Pose2d llPose, Pose2d currentPose)
  {
    Transform2d deltaTransform = currentPose.minus(llPose);

    return Math.hypot(deltaTransform.getX( ), deltaTransform.getY( )) < 1.0;
  }

  public void setLEDMode(int mode)
  {
    DataLogManager.log(String.format("%s: setLedMode %d", getSubsystem( ), mode));
    m_table.getEntry("ledMode").setValue(mode);
  }

  public void setCameraDisplay(int stream)
  {
    DataLogManager.log(String.format("%s: setCameraDisplay %d", getSubsystem( ), stream));
    m_table.getEntry("stream").setValue(stream);
  }
}

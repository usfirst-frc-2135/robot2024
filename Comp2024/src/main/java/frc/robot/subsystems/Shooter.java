//
// Shooter Subystem - scores Notes into the Speaker
//
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SHConsts.ShooterMode;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.PhoenixUtil6;

//
// Shooter subsystem class
//
public class Shooter extends SubsystemBase
{
  // Constants
  public static final double kFlywheelGearRatio      = (18.0 / 18.0);

  public static final double kFlywheelToleranceRPM   = 150.0;     // Tolerance band around target RPM
  public static final double kFlywheelUpperTargetRPM = 2150.0;    // RPM to score

  // Devices and simulation objects
  private final TalonFX      m_shooterLower          = new TalonFX(Ports.kCANID_ShooterLower);
  private final TalonFX      m_shooterUpper          = new TalonFX(Ports.kCANID_ShooterUpper);

  // Declare module variables
  private boolean            m_valid                 = false; // Health indicator for shooter talon 11
  private boolean            m_ifShooterTest         = false; // checks to see if testing the shooter
  private boolean            m_atDesiredSpeed        = false; // Indicates flywheel RPM is close to target

  private ShooterMode        m_curMode;               // Current shooter mode
  private double             m_targetSpeed;           // Requested flywheel RPM
  private double             m_flywheelRPM;           // Current flywheel RPM

  // Constructor

  public Shooter( )
  {
    setName("Shooter");
    setSubsystem("Shooter");

    m_valid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_shooterLower, "Lower", CTREConfigs6.shooterFXConfig( ))
        && PhoenixUtil6.getInstance( ).talonFXInitialize6(m_shooterLower, "Upper", CTREConfigs6.shooterFXConfig( ));
    m_shooterUpper.setControl(new Follower(m_shooterLower.getDeviceID( ), true));

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  public void initialize( )
  {
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
    setShooterMode(ShooterMode.STOP);
  }

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }

  public void setShooterMode(ShooterMode mode)
  {
    m_curMode = mode;

    DataLogManager.log(getSubsystem( ) + ": set shooter mode " + mode);

    // Select the shooter RPM from the requested mode

    switch (mode)
    {
      case STOP :
        m_targetSpeed = 0.0;
        break;
      case REVERSE :
        m_targetSpeed = -0.5;
        break;
      case SCORE :
        m_targetSpeed = 0.5;
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": invalid shooter mode requested " + mode);
        break;
    }

    if (m_valid)
      m_shooterLower.set(m_targetSpeed);
    DataLogManager.log(getSubsystem( ) + ": target speed is " + m_targetSpeed);
  }

}

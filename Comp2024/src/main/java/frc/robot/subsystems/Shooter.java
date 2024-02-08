//
// Shooter Subystem - scores Notes into the Speaker
//
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SHConsts.SHMode;
import frc.robot.lib.util.PhoenixUtil6;

//
// Shooter subsystem class
//
public class Shooter extends SubsystemBase
{
  // Constants

  // Devices and simulation objects
  private final TalonFX        m_shooterL       = new TalonFX(Ports.kCANID_ShooterL);
  private final TalonFX        m_shooterR       = new TalonFX(Ports.kCANID_ShooterR);

  private TalonFXConfiguration config           = new TalonFXConfiguration( );

  // Declare module variables
  private boolean              m_valid          = false; // Health indicator for shooter talon 11
  private boolean              m_ifShooterTest  = false; // checks to see if testing the shooter
  private boolean              m_atDesiredSpeed = false; // Indicates flywheel RPM is close to target
  private boolean              m_atDesiredSpeedPrevious;

  private SHMode               m_curMode;                // Current shooter mode
  private double               m_targetSpeed;      // Requested flywheel RPM
  private double               m_flywheelRPM;            // Current flywheel RPM

  //Devices and simulation objs

  // Constructor
  public Shooter( )
  {
    setName("Shooter");
    setSubsystem("Shooter");

    m_valid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_shooterL, "Left", config)
        && PhoenixUtil6.getInstance( ).talonFXInitialize6(m_shooterR, "Right", config);

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
    setShooterMode(SHMode.SHOOTER_STOP);
  }

  public void setShooterMode(SHMode mode)
  {
    m_curMode = mode;

    DataLogManager.log(getSubsystem( ) + ": set shooter mode " + mode);

    // Select the shooter RPM from the requested mode

    switch (mode)
    {
      case SHOOTER_STOP :
        m_targetSpeed = 0.0;
        break;
      case SHOOTER_REVERSE :
        m_targetSpeed = -0.5;
        break;
      case SHOOTER_SCORE :
        m_targetSpeed = 0.5;
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": invalid shooter mode requested " + mode);
        break;
    }

    if (m_valid)
    {
      m_shooterL.set(m_targetSpeed);
      m_shooterR.set(-m_targetSpeed);
    }
    DataLogManager.log(getSubsystem( ) + ": target speed is " + m_targetSpeed);
  }

}

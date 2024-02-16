//
// Shooter Subystem - scores Notes into the Speaker
//
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SHConsts.ShooterMode;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREConfigs6;
import frc.robot.lib.util.PhoenixUtil6;

//
// Shooter subsystem class
//
public class Shooter extends SubsystemBase
{
  // Constants
  public static final double    kFlywheelGearRatio      = (18.0 / 18.0);

  public static final double    kFlywheelToleranceRPM   = 150.0;     // Tolerance band around target RPM
  public static double          kFlywheelLowerTargetRPM = 2150.0;    // RPM to score

  // Devices and simulation objects
  private final TalonFX         m_shooterLower          = new TalonFX(Ports.kCANID_ShooterLower);
  private final TalonFX         m_shooterUpper          = new TalonFX(Ports.kCANID_ShooterUpper);

  private final TalonFXSimState m_shooterLowerSim       = new TalonFXSimState(m_shooterLower);
  private final TalonFXSimState m_shooterUpperSim       = new TalonFXSimState(m_shooterUpper);
  private final FlywheelSim     m_flywheelLowerSim      = new FlywheelSim(DCMotor.getFalcon500(1), kFlywheelGearRatio, 0.005);
  private final FlywheelSim     m_flywheelUpperSim      = new FlywheelSim(DCMotor.getFalcon500(1), kFlywheelGearRatio, 0.005);

  private VelocityVoltage       m_requestVelocity       = new VelocityVoltage(0.0);
  private LinearFilter          m_flywheelFilter        = LinearFilter.singlePoleIIR(0.1, 0.02);

  // Declare module variables
  private static boolean        m_isComp;

  private boolean               m_shooterValid;
  private boolean               m_atDesiredSpeed        = false; // Indicates flywheel RPM is close to target
  private boolean               m_atDesiredSpeedPrevious;

  private double                m_flywheelTargetRPM;     // Requested flywheel RPM
  private double                m_flywheelRPM;           // Current flywheel RPM

  // Constructor

  public Shooter(boolean isComp)
  {
    setName("Shooter");
    setSubsystem("Shooter");
    m_isComp = isComp;

    m_shooterValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_shooterLower, "Lower", CTREConfigs6.shooterFXConfig( ))
        && PhoenixUtil6.getInstance( ).talonFXInitialize6(m_shooterUpper, "Upper", CTREConfigs6.shooterFXConfig( ));
    m_shooterUpper.setControl(new Follower(m_shooterLower.getDeviceID( ), true));

    initSmartDashboard( );
    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    double current = 0.0;

    // Calculate flywheel RPM and display on dashboard
    if (m_shooterValid)
    {
      m_flywheelRPM = m_flywheelFilter.calculate((m_shooterLower.getVelocity( ).refresh( ).getValue( ) * 60.0));

      m_atDesiredSpeed = (m_flywheelRPM > 30.0) && (Math.abs(m_flywheelTargetRPM - m_flywheelRPM) < kFlywheelToleranceRPM);

      if (m_atDesiredSpeed != m_atDesiredSpeedPrevious)
      {
        DataLogManager.log(getSubsystem( ) + ": at desired speed now " + m_flywheelTargetRPM);
        m_atDesiredSpeedPrevious = m_atDesiredSpeed;
      }

      current = m_shooterLower.getStatorCurrent( ).refresh( ).getValue( );
    }

    SmartDashboard.putNumber("SH_flywheelRPM", m_flywheelRPM);
    SmartDashboard.putBoolean("SH_atDesiredSpeed", m_atDesiredSpeed);
    SmartDashboard.putNumber("SH_current", current);
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input flywheel voltage from the motor setting
    m_shooterLowerSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_shooterUpperSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_flywheelLowerSim.setInput(m_shooterLowerSim.getMotorVoltage( ));
    m_flywheelUpperSim.setInput(m_shooterUpperSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_flywheelLowerSim.update(0.020);
    m_flywheelUpperSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_shooterLowerSim.setRotorVelocity(m_flywheelLowerSim.getAngularVelocityRPM( ) / 60.0);
    m_shooterUpperSim.setRotorVelocity(m_flywheelUpperSim.getAngularVelocityRPM( ) / 60.0);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelLowerSim.getCurrentDrawAmps( )));
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelUpperSim.getCurrentDrawAmps( )));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  private void initSmartDashboard( )
  {
    SmartDashboard.putBoolean("HL_SHValid", m_shooterValid);

    SmartDashboard.putNumber("SH_targetRPM", kFlywheelLowerTargetRPM);
  }

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
    DataLogManager.log(getSubsystem( ) + ": set shooter mode " + mode);

    kFlywheelLowerTargetRPM = SmartDashboard.getNumber("SH_targetRPM", kFlywheelLowerTargetRPM);

    // Select the shooter RPM for the requested mode - NEVER NEGATIVE when running!
    switch (mode)
    {
      case STOP :
        m_flywheelRPM = 0.0;
        break;
      case SCORE :
        m_flywheelRPM = kFlywheelLowerTargetRPM;
        break;
      default :
        DataLogManager.log(getSubsystem( ) + ": invalid shooter mode requested " + mode);
        break;
    }

    if (m_shooterValid)
      m_shooterLower.setControl(
          m_requestVelocity.withVelocity(Conversions.rotationsToInputRotations(m_flywheelRPM / 60.0, kFlywheelGearRatio)));
    DataLogManager.log(getSubsystem( ) + ": target speed is " + m_flywheelRPM);
  }

  public boolean isAtDesiredSpeed( )
  {
    return m_atDesiredSpeed;
  }
}

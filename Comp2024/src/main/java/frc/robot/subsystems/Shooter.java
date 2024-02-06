//
// Shooter Subystem - scores Notes into the Speaker
//
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

//
// Shooter subsystem class
//
public class Shooter extends SubsystemBase
{
  // Member objects
  private final TalonFX m_shooterL = new TalonFX(Ports.kCANID_ShooterL);
  private final TalonFX m_shooterR = new TalonFX(Ports.kCANID_ShooterR);

  //Devices and simulation objs

  // Constructor
  public Shooter( )
  {
    setName("Shooter");
    setSubsystem("Shooter");
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
  {}

  public void faultDump( )
  {
    DataLogManager.log(String.format("%s: faultDump  ----- DUMP FAULTS --------------", getSubsystem( )));
  }
}

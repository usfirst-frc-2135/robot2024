//
// Shooter Subystem - scores Notes into the Speaker
//
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//
// Shooter subsystem class
//
public class Shooter extends SubsystemBase
{
  // Member objects
  private final TalonFX m_shooter = new TalonFX(0);

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
}
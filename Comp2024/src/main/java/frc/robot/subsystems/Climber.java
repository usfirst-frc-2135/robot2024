//
// Intake Subystem - takes in Notes and delivers them to the Shooter
//
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//
// Intake subsystem class
//
public class Climber extends SubsystemBase
{
  // Member objects
  private final TalonFX m_climberL = new TalonFX(0);
  private final TalonFX m_climberR = new TalonFX(0);

  //Devices and simulation objs

  // Constructor
  public Climber( )
  {
    setName("Climber");
    setSubsystem("Climber");
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

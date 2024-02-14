//
// Climber Subystem - lifts the robot to hang onto the chain
//
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

//
// Climber subsystem class
//
public class Climber extends SubsystemBase
{
  // Member objects
  private final TalonFX  m_climberL = new TalonFX(Ports.kCANID_ClimberL);
  private final TalonFX  m_climberR = new TalonFX(Ports.kCANID_ClimberR);

  // Devices and simulation objs

  // Declare module variables
  private static boolean m_isComp;

  // Constructor

  public Climber(boolean isComp)
  {
    setName("Climber");
    setSubsystem("Climber");
    m_isComp = isComp;

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
